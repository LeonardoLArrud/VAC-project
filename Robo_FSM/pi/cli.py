import queue
import re
import sys
import threading
import time

from rich.console import Console, Group
from rich.panel import Panel
from rich.prompt import Prompt
from rich.table import Table
from rich.text import Text

HEADINGS = "ENWS"                   # navigators.Mission: E, N, W, S
ARROW = {"E": "▶", "N": "▲", "W": "◀", "S": "▼"}
CAR = "bold black on bright_green"      # the robot on the grid: a bright block, easy to spot
GOAL = "bold white on bright_magenta"   # the destination
QUIT = "q"
DEFAULT_N, DEFAULT_G = 5, 0.975     # nodes per side, m between node centres (the track)
EDGE_S = 1.0                        # --fake: seconds to drive one edge

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
        row = Text()                    # no style on the row itself: the car and goal would inherit it (dim)
        row.append(f"{y} ", style="dim")
        for x in range(n):
            if (x, y) == robot:
                row.append(ARROW[heading], style=CAR)
            elif (x, y) == goal:
                row.append("◎", style=GOAL)
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
                           title=f"[{CAR}] {self.node} [/] → [{GOAL}] {goal} [/]", subtitle="q + Enter: stop", expand=False))
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


def cli_hardware(base, cli):
    """The robot's hardware class (RobotHardware, WebotsHardware) with the commands coming from the CLI.
    A subclass, not a wrapper: WebotsHardware.read() calls self.command() itself."""

    class CliHardware(base):
        def command(self):              # the CLI first; then the class's own (Webots keys G, S, L)
            return cli.command() or super().command()

        def step(self):                 # False after q: fsm.main's loop ends (the motors stop on exit)
            return cli.running() and super().step()

        def attach(self, fsm):
            cli.attach(fsm)

    return CliHardware


class FakeFsm:
    """--fake: plays the FSM's part on the PC, without camera or wheels: drives the Mission node by node,
    EDGE_S seconds per edge, with the same calls as the real FSM (Mission.on_node at each node)."""

    def __init__(self):
        self.state, self.nav, self.reason, self.t_next = "IDLE", None, "start", 0.0

    def step(self, cmd, t):
        from navigators import Mission
        if cmd == "STOP":
            self.state, self.reason = "IDLE", "STOP command"
        elif isinstance(cmd, tuple) and self.state == "IDLE":
            self.nav, self.state, self.t_next = Mission(*cmd[1:]), "NODE", t
        if self.state == "NODE" and t >= self.t_next:       # the next node is reached
            order, why = self.nav.on_node(None)
            console.print(f"[dim]  fake robot: {order.lower()}: {why}[/]")
            if order == "STOP":
                self.state, self.reason = "IDLE", f"STOP at the node: {why}"
            self.t_next = t + EDGE_S


def run_fake(cli, time_step=0.064):
    fsm = FakeFsm()
    cli.attach(fsm)
    while cli.running():                # the robot loop, 64 ms per cycle
        fsm.step(cli.command(), time.monotonic())
        time.sleep(time_step)


def ask_start(n):
    """Start node and heading typed by the user (MISSION = None in params.ini, or --fake)."""
    while True:
        start = parse_node(Prompt.ask("Start node x y", default="0 0"), n)
        if start is not None:
            break
        console.print(f"[red]type x y, each between 0 and {n - 1}[/]")
    heading = Prompt.ask("Heading", choices=list(HEADINGS), default="E")
    return start, heading


def start_cli(start, heading, n, g):
    """Header, start view and the keyboard thread. Returns the DeliveryCli the robot loop polls."""
    console.print(Panel("[bold]ST5 Troller: deliveries on the grid[/]\n"
                        "x right, y up, (0, 0) bottom-left. Type a destination, Enter; q to quit.",
                        border_style="cyan", expand=False))
    cli = DeliveryCli(start, heading, n, g)
    console.print(Panel(grid_view(n, cli.node, cli.heading), title=f"[bold]start {cli.node}, facing {heading}[/]",
                        expand=False))
    threading.Thread(target=cli.input_loop, daemon=True).start()
    return cli


def main(argv=None):
    """python3 cli.py: the real robot (Pi). python3 cli.py --fake: on a PC, a fake robot (no camera, no serial)."""
    argv = sys.argv[1:] if argv is None else argv
    if "--fake" in argv:
        start, heading = ask_start(DEFAULT_N)
        cli = start_cli(start, heading, DEFAULT_N, DEFAULT_G)
        try:
            run_fake(cli)
        except KeyboardInterrupt:
            console.print("\nstopped (Ctrl+C)")
        console.print(cli.summary())
        return
    import fsm                          # robot code (camera, serial): imported here, only on the Pi
    from robot_hardware import RobotHardware

    if fsm.MISSION is not None:         # params.ini: start, heading, grid size and spacing from the mission
        _, start, heading, _, n, g = fsm.MISSION
    else:
        n, g = DEFAULT_N, DEFAULT_G
        start, heading = ask_start(n)
    fsm.MISSION = ("MISSION", tuple(start), heading, tuple(start), n, g)   # not None: the node detectors start
    fsm.AUTO_START = None               # no automatic mission: the CLI gives them
    cli = start_cli(start, heading, n, g)
    try:
        fsm.main(cli_hardware(RobotHardware, cli)(fsm.TIME_STEP))
    except KeyboardInterrupt:           # Ctrl+C still works (the motors stop on exit)
        console.print("\nstopped (Ctrl+C)")
    console.print(cli.summary())


if __name__ == "__main__":
    main()
