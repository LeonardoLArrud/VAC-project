"""Deliveries on the grid, chosen while the robot runs: type a destination, the robot drives there, then type the
next one, and so on until q. Run on the Pi: python3 cli.py (needs rich: pip install rich).

The FSM and the navigation do not change: this only sends ("MISSION", ...) commands through hw.command() and
watches the FSM (hw.attach) to know when a delivery ends. rich prints only at events (new destination, arrival),
never in every cycle of the loop, so the 64 ms loop is not slowed.
"""
import queue
import re
import threading
import time

from rich.console import Console, Group
from rich.panel import Panel
from rich.prompt import Prompt
from rich.table import Table
from rich.text import Text

HEADINGS = "ENWS"                   # navigators.Mission: E, N, W, S
ARROW = {"E": "▶", "N": "▲", "W": "◀", "S": "▼"}
QUIT = "q"
DEFAULT_N, DEFAULT_G = 5, 0.975     # nodes per side, m between node centres (the track)

console = Console()


def parse_node(text, n):
    """'3 2', '3,2' or '(3, 2)' -> (3, 2) if inside the n x n grid, else None."""
    numbers = re.findall(r"-?\d+(?:\.\d+)?", text)
    if len(numbers) != 2 or any("." in s for s in numbers):
        return None
    x, y = map(int, numbers)
    return (x, y) if 0 <= x < n and 0 <= y < n else None


def grid_view(n, robot, heading, goal=None):
    """The grid seen from above, y up: the robot as an arrow, the goal as ◎, the other nodes as ·."""
    lines = []
    for y in range(n - 1, -1, -1):
        row = Text(f"{y} ", style="dim")
        for x in range(n):
            if (x, y) == robot:
                row.append(ARROW[heading], style="bold green")
            elif (x, y) == goal:
                row.append("◎", style="bold yellow")
            else:
                row.append("·", style="dim")
            if x < n - 1:
                row.append(" ─ ", style="dim")
        lines.append(row)
        if y:
            lines.append(Text("  " + "   ".join("│" * n), style="dim"))
    lines.append(Text("  " + "   ".join(str(x) for x in range(n)), style="dim"))
    return Group(*lines)


class DeliveryCli:
    """Between the keyboard (input thread) and the robot loop (command(), every cycle)."""

    def __init__(self, start, heading, n=DEFAULT_N, g=DEFAULT_G, ask=console.input, say=console.print):
        self.node, self.heading, self.n, self.g = tuple(start), heading, n, g
        self.goal = None                # destination being driven to (None: waiting for one)
        self.sent = False               # its MISSION command was handed to the robot loop
        self.t_goal = 0.0
        self.fsm = None
        self.pending = queue.Queue()    # commands for the robot loop
        self.quit = False
        self.deliveries = []            # (goal, result, seconds)
        self.ask, self.say = ask, say

    def attach(self, fsm):
        self.fsm = fsm

    def running(self):
        return not self.quit

    # ------------------------------------------------------------ keyboard side (input thread)
    def handle(self, text):
        """One line typed by the user. Returns False after q."""
        text = text.strip().lower()
        if text == QUIT:
            self.quit = True
            self.pending.put("STOP")
            return False
        if self.goal is not None:
            self.say(f"[yellow]driving to {self.goal}: wait until it arrives, or q to stop[/]")
            return True
        goal = parse_node(text, self.n)
        if goal is None:
            self.say(f"[red]'{text}' is not a node: type x y, each between 0 and {self.n - 1}[/]")
        elif goal == self.node:
            self.say(f"[yellow]already at {goal}[/]")
        else:
            self.goal, self.sent, self.t_goal = goal, False, time.monotonic()
            self.pending.put(("MISSION", self.node, self.heading, goal, self.n, self.g))
            self.say(Panel(grid_view(self.n, self.node, self.heading, goal),
                           title=f"[bold]{self.node} → {goal}[/]", subtitle="q + Enter: stop", expand=False))
        return True

    def input_loop(self):
        self.say(self.next_prompt())
        while not self.quit:
            try:
                text = self.ask("> ")
            except (EOFError, KeyboardInterrupt):
                text = QUIT
            if not self.handle(text):
                break

    def next_prompt(self):
        return f"[bold cyan]Next destination: x y (0..{self.n - 1}), q to quit[/]"

    # ------------------------------------------------------------ robot side (called every cycle)
    def command(self):
        """The next command for the FSM (None most cycles). Also notices when a delivery has ended."""
        self.check_end()
        try:
            cmd = self.pending.get_nowait()
        except queue.Empty:
            return None
        if isinstance(cmd, tuple):
            self.sent = True
        return cmd

    def check_end(self):
        """A MISSION was sent and the FSM is back in IDLE: the robot stopped (at the goal, or not)."""
        fsm = self.fsm
        if not self.sent or fsm is None or fsm.state != "IDLE" or self.quit:
            return
        nav = fsm.nav
        self.node, self.heading = tuple(nav.node), HEADINGS[nav.h_in]   # where it stopped, facing
        arrived = self.node == self.goal
        result = "arrived" if arrived else fsm.reason
        self.deliveries.append((self.goal, result, time.monotonic() - self.t_goal))
        style, title = ("green", f"arrived at {self.node}") if arrived else ("red", f"stopped at {self.node}")
        self.say(Panel(Group(grid_view(self.n, self.node, self.heading),
                             Text(result if not arrived else f"in {self.deliveries[-1][2]:.0f} s")),
                       title=f"[bold {style}]{title}[/]", border_style=style, expand=False))
        self.say(self.next_prompt())
        self.goal, self.sent = None, False

    def summary(self):
        table = Table(title="Deliveries")
        for col in ("#", "destination", "result", "time"):
            table.add_column(col)
        for i, (goal, result, secs) in enumerate(self.deliveries, 1):
            table.add_row(str(i), str(goal), result, f"{secs:.0f} s",
                          style="green" if result == "arrived" else "red")
        return table


class CliHardware:
    """The robot's hardware, with the commands coming from the CLI. Everything else is RobotHardware's."""

    def __init__(self, hw, cli):
        self.hw, self.cli = hw, cli

    def __getattr__(self, name):        # camera, wheels, sonar, show... unchanged
        return getattr(self.hw, name)

    def command(self):
        return self.cli.command()

    def step(self):                     # False after q: fsm.main's loop ends (the motors stop on exit)
        return self.cli.running() and self.hw.step()

    def attach(self, fsm):
        self.cli.attach(fsm)


def ask_start(n):
    """Start node and heading typed by the user (MISSION = None in params.ini)."""
    while True:
        start = parse_node(Prompt.ask("Start node x y", default="0 0"), n)
        if start is not None:
            break
        console.print(f"[red]type x y, each between 0 and {n - 1}[/]")
    heading = Prompt.ask("Heading", choices=list(HEADINGS), default="E")
    return start, heading


def main():
    import fsm                          # robot code (camera, serial): imported here, only on the Pi
    from robot_hardware import RobotHardware

    console.print(Panel("[bold]ST5 Troller: deliveries on the grid[/]\n"
                        "x right, y up, (0, 0) bottom-left. Type a destination, Enter; q to quit.",
                        border_style="cyan", expand=False))
    if fsm.MISSION is not None:         # params.ini: start, heading, grid size and spacing from the mission
        _, start, heading, _, n, g = fsm.MISSION
    else:
        n, g = DEFAULT_N, DEFAULT_G
        start, heading = ask_start(n)
    fsm.MISSION = ("MISSION", tuple(start), heading, tuple(start), n, g)   # not None: the node detectors start
    fsm.AUTO_START = None               # no automatic mission: the CLI gives them
    cli = DeliveryCli(start, heading, n, g)
    console.print(Panel(grid_view(n, cli.node, cli.heading), title=f"[bold]start {cli.node}, facing {heading}[/]",
                        expand=False))
    threading.Thread(target=cli.input_loop, daemon=True).start()
    try:
        fsm.main(CliHardware(RobotHardware(fsm.TIME_STEP), cli))
    except KeyboardInterrupt:           # Ctrl+C still works (the motors stop on exit)
        console.print("\nstopped (Ctrl+C)")
    console.print(cli.summary())


if __name__ == "__main__":
    main()
