import ast
import configparser
import os
import queue
import re
import sys
import tempfile
import threading
import time

from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.prompt import FloatPrompt, Prompt
from rich.table import Table
from rich.text import Text

HEADINGS = "ENWS"                   # navigators.Mission: E, N, W, S
ARROW = {"E": "▶", "N": "▲", "W": "◀", "S": "▼"}
CAR = "bold bright_green"           # the robot on the grid
GOAL = "bold bright_yellow"         # the destination
PATH = "bold bright_cyan"           # --lp: the Dijkstra path
BLOCKED = "bold red"                # --lp: edges blocked by an obstacle
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


def grid_view(n, robot, heading, goal=None, path=(), blocked=()):
    """The grid seen from above, y up: the robot as an arrow, the goal as ◎, the other nodes as ·.
    path: nodes [(x, y), ...] whose edges are highlighted; blocked: edges ((x, y), (x, y)) drawn as ✗."""
    on_path = {frozenset(e) for e in zip(path, path[1:])}
    closed = {frozenset(e) for e in blocked}

    def edge(a, b, plain, along):       # how to draw the edge a-b
        e = frozenset((a, b))
        if e in closed:
            return (" ✗ " if plain == " ─ " else "✗"), BLOCKED
        return (along, PATH) if e in on_path else (plain, "dim")

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
                row.append("·", style=PATH if (x, y) in path else "dim")
            if x < n - 1:
                row.append(*edge((x, y), (x + 1, y), " ─ ", "━━━"))
        lines.append(row)
        if y:
            column = Text("  ")
            for x in range(n):
                column.append(*edge((x, y), (x, y - 1), "│", "┃"))
                if x < n - 1:
                    column.append("   ")
            lines.append(column)
    lines.append(Text("  " + "   ".join(str(x) for x in range(n)), style="dim"))
    return Group(*lines)


# ------------------------------------------------------------ --lp: live preview while the robot drives
def preview_path(nav, n):
    """After an obstacle Mission empties its path until the robot is back on a node: the path it will take from
    there (the same Dijkstra on the same map), to show the replan at once. (x, y) nodes, [] if none."""
    from graph.path import dijkstra    # found through navigators (already imported by the FSM)
    ids, _ = dijkstra(nav.map, (n - 1 - nav.to[1], nav.to[0]), (n - 1 - nav.goal[1], nav.goal[0]))
    path = [(c, n - 1 - r) for r, c in ids]
    return [tuple(nav.node)] + path if path and tuple(nav.node) != tuple(nav.to) else path


def still_blocked(nav, n):
    """The edges of nav.closed the map still blocks: they expire after Mission's t_block, and a boxed-in robot
    forgets them all; nav.closed keeps the history."""
    def grid_id(xy):                    # (x, y) -> graph.track (row, col), as Mission._id
        return n - 1 - xy[1], xy[0]
    return {tuple(sorted(map(tuple, e))) for e in nav.closed if nav.map.is_blocked(grid_id(e[0]), grid_id(e[1]))}


def snapshot(nav, n):
    """What the live panel shows, read from the Mission: (node, heading, next, path, blocked edges, replanning).
    Replanning: Mission's path is empty (after an obstacle, until the node behind) or stops short of the goal
    (the sonar saw an obstacle beyond the next node: on_seen keeps only [node, next]). Then the path it will take
    from the next node is shown at once."""
    path, replanning = [tuple(p) for p in nav.path], False
    if not path or path[-1] != tuple(nav.goal):
        path, replanning = preview_path(nav, n), True
    return (tuple(nav.node), HEADINGS[nav.h], tuple(nav.to), tuple(path), tuple(sorted(still_blocked(nav, n))),
            replanning)


def live_panel(n, start, goal, snap):
    node, heading, to, path, blocked, replanning = snap
    left = len(path) - 1
    if replanning:
        status = f"[{BLOCKED}]obstacle[/]: back to {to}, new path {left} edges" if path else \
                 f"[{BLOCKED}]obstacle[/]: back to {to}, no path to {goal}"
    elif node == tuple(goal):
        status = f"at the goal {node}: stopping on the node"
    else:
        status = f"at {node} → next {to} · {left} edge{'s' * (left != 1)} left"
    legend = Text.from_markup(f"[{CAR}]{ARROW[heading]}[/] car  [{GOAL}]◎[/] goal  [{PATH}]━[/] path  "
                              f"[{BLOCKED}]✗[/] blocked")
    title = f"[{CAR}]{start}[/] → [{GOAL}]{goal}[/]" + (f" · [{BLOCKED}]replanned[/]" if blocked else "")
    return Panel(Group(grid_view(n, node, heading, goal, path, blocked), Text.from_markup(status), legend),
                 title=title, subtitle="q: stop", expand=False)


class LivePreview:
    """--lp: while the robot drives, one panel redrawn in place. The robot loop only queues a snapshot when
    something changed (microseconds); this thread draws it with the render function given at start. Off between runs."""

    def __init__(self, out=None):
        self.console = out or console
        self.events = queue.Queue()
        self.idle = threading.Event()   # set: no live panel, the prompt can be used
        self.idle.set()
        self.render = None
        threading.Thread(target=self._run, daemon=True).start()

    def start(self, render, waiting):   # keyboard thread: a run begins. render(snapshot) -> what to draw
        self.idle.clear()
        self.render = render
        self.events.put(("start", waiting))

    def update(self, snap):             # robot loop: a new snapshot (only when it changed)
        self.events.put(("update", snap))

    def end(self, *renderables):        # robot loop: the delivery ended; printed once the panel is off
        self.events.put(("end", renderables))

    def close(self, timeout=2.0):       # at exit: leave the terminal clean
        self.events.put(("end", ()))
        self.idle.wait(timeout)

    def _run(self):
        live = None
        while True:
            kind, data = self.events.get()
            if kind == "start":
                live = Live(console=self.console, auto_refresh=False, redirect_stdout=True, redirect_stderr=True,
                            transient=True)   # the panel goes away at the end: the result replaces it
                live.start()
                live.update(Text(data), refresh=True)
            elif kind == "update" and live is not None:
                live.update(self.render(data), refresh=True)
            elif kind == "end":
                if live is not None:
                    live.stop()
                    live = None
                for r in data:
                    self.console.print(r)
                self.idle.set()


def wait_for_q(cli, poll=0.1):
    """--lp while driving: q alone (no Enter) stops at once. Returns when the run has ended (cli.busy() False), or
    after q. Other keys go to cli.key() if it has one (--fake --eight: o = obstacle), else are dropped."""
    if os.name == "nt":
        return _wait_for_q_windows(cli, poll)
    import select
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd) if sys.stdin.isatty() else None
    try:
        if old is not None:
            tty.setcbreak(fd)           # one key at a time, no Enter
        while cli.busy() and not cli.quit:
            if select.select([sys.stdin], [], [], poll)[0]:
                key = sys.stdin.read(1)
                if key == "" or key.lower() == QUIT:   # "" = end of input (e.g. piped)
                    cli.handle(QUIT)
                    return
                if hasattr(cli, "key"):
                    cli.key(key.lower())
    finally:
        if old is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _wait_for_q_windows(cli, poll):
    """wait_for_q on Windows, which has no termios: msvcrt reads the console one key at a time."""
    import msvcrt
    while cli.busy() and not cli.quit:
        if not msvcrt.kbhit():
            time.sleep(poll)
            continue
        key = msvcrt.getwch().lower()
        if key == QUIT:
            cli.handle(QUIT)
            return
        if hasattr(cli, "key"):
            cli.key(key)


class DeliveryCli:
    """Between the keyboard (input thread) and the robot loop (command(), every cycle)."""

    def __init__(self, start, heading, n=DEFAULT_N, g=DEFAULT_G, ask=console.input, say=console.print,
                 live=False, on_block=None):
        self.node, self.heading, self.n, self.g = tuple(start), heading, n, g
        self.goal = None                # destination being driven to (None: waiting for one)
        self.sent = False               # its MISSION command was handed to the robot loop
        self.t_goal = 0.0
        self.fsm = None
        self.pending = queue.Queue()    # commands for the robot loop
        self.quit = False
        self.deliveries = []            # (goal, result, seconds)
        self.lost = False               # the last delivery stopped short: where the robot is must be typed first
        self.ask, self.say = ask, say
        self.live = LivePreview() if live else None      # --lp
        self.live_key = None            # what the live panel last showed: redrawn only when it changes
        self.on_block = on_block        # --fake: "block x1 y1 x2 y2" puts a hidden obstacle on that edge

    def attach(self, fsm):
        self.fsm = fsm

    def running(self):
        return not self.quit

    def busy(self):                     # a delivery is under way
        return self.goal is not None

    # ------------------------------------------------------------ keyboard side (input thread)
    def handle(self, text):
        """One line typed by the user. Returns False after q."""
        text = text.strip().lower()
        if text == QUIT:
            if self.goal is not None:   # stopped on the way: it goes in the summary too
                self.deliveries.append((self.goal, "stopped (q)", time.monotonic() - self.t_goal))
            self.quit = True
            self.pending.put("STOP")
            return False
        if self.on_block and text.startswith("block"):
            self.block(text)
            return True
        if self.goal is not None:
            self.say(f"[yellow]driving to {self.goal}: wait until it arrives, or q to stop[/]")
            return True
        if self.lost:
            self.relocate(text)
            return True
        goal = parse_node(text, self.n)
        if goal is None:
            self.say(f"[red]'{text}' is not a node: type x y, each between 0 and {self.n - 1}[/]")
        elif goal == self.node:
            self.say(f"[yellow]already at {goal}[/]")
        else:
            self.goal, self.sent, self.t_goal = goal, False, time.monotonic()
            if self.live:
                self.live_key = None
                self.live.start(lambda snap, start=self.node: live_panel(self.n, start, goal, snap),
                                f"to {goal}: starting...")
            else:
                self.say(Panel(grid_view(self.n, self.node, self.heading, goal),
                               title=f"[{CAR}]{self.node}[/] → [{GOAL}]{goal}[/]", subtitle="q + Enter: stop",
                               expand=False))
            self.pending.put(("MISSION", self.node, self.heading, goal, self.n, self.g))
        return True

    def relocate(self, text):
        """After a delivery that stopped short: where the robot really is, before the next destination.
        'x y heading' sets it; Enter keeps the last node the Mission was sure of."""
        if text:
            m = re.fullmatch(r"(.*?)[\s,]*([enws])", text)
            node = parse_node(m.group(1), self.n) if m else None
            if node is None:
                self.say(f"[red]type x y heading, e.g. 1 1 s (Enter keeps {self.node} {self.heading})[/]")
                return
            self.node, self.heading = node, m.group(2).upper()
        self.lost = False
        self.say(Panel(grid_view(self.n, self.node, self.heading), title=f"[{CAR}]robot at {self.node}, facing "
                       f"{self.heading}[/]", expand=False))
        self.say(self.next_prompt())

    def block(self, text):
        """--fake: 'block x1 y1 x2 y2' -> a hidden obstacle on the edge between two neighbor nodes."""
        numbers = re.findall(r"-?\d+", text)
        a = parse_node(" ".join(numbers[:2]), self.n) if len(numbers) == 4 else None
        b = parse_node(" ".join(numbers[2:]), self.n) if len(numbers) == 4 else None
        if a is None or b is None or abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
            self.say("[red]type block x1 y1 x2 y2, two neighbor nodes of the grid[/]")
            return
        self.on_block(frozenset((a, b)))
        self.say(f"fake obstacle on {a}-{b}: the robot finds it when it gets there")

    def input_loop(self):
        self.say(self.next_prompt())
        while not self.quit:
            if self.live:
                self.live.idle.wait()   # the live panel is off: the prompt can be used
            try:
                text = self.ask("> ")
            except (EOFError, KeyboardInterrupt):
                text = QUIT
            if not self.handle(text):
                break
            if self.live and self.busy():
                wait_for_q(self)        # driving: only q, no Enter

    def next_prompt(self):
        if self.lost:
            return (f"[bold yellow]Where is the robot now? x y heading, e.g. 1 1 S (Enter: {self.node} "
                    f"{self.heading}, the last node it was sure of)[/]")
        extra = ", block x1 y1 x2 y2: fake obstacle" if self.on_block else ""
        stop = "" if not self.live else " (while driving: q alone stops)"
        return f"[bold cyan]Next destination: x y (0..{self.n - 1}), q to quit{extra}{stop}[/]"

    def close(self):
        if self.live:
            self.live.close()

    # ------------------------------------------------------------ robot side (called every cycle)
    def command(self):
        """The next command for the FSM (None most cycles). Also notices when a delivery has ended."""
        self.check_end()
        fsm = self.fsm
        if self.live and self.sent and fsm is not None and fsm.state != "IDLE" and hasattr(fsm.nav, "path"):
            self.watch(fsm.nav)         # a delivery is running: the live panel follows its Mission
        try:
            cmd = self.pending.get_nowait()
        except queue.Empty:
            return None
        if isinstance(cmd, tuple):
            self.sent = True
        return cmd

    def watch(self, nav):               # every cycle of a delivery: a snapshot only when something changed
        key = (nav.node, nav.to, nav.h, len(nav.path), len(nav.closed), len(still_blocked(nav, self.n)))
        if key != self.live_key:
            self.live_key = key
            self.live.update(snapshot(nav, self.n))

    def check_end(self):
        """A MISSION was sent and the FSM is back in IDLE: the robot stopped (at the goal, or not)."""
        fsm = self.fsm
        if not self.sent or fsm is None or fsm.state != "IDLE" or self.quit:
            return
        nav = fsm.nav
        self.node, self.heading = tuple(nav.node), HEADINGS[nav.h_in]   # where it stopped, facing
        arrived = self.node == self.goal
        self.lost = not arrived         # the next line typed is where it is (Enter: here)
        result = "arrived" if arrived else fsm.reason
        self.deliveries.append((self.goal, result, time.monotonic() - self.t_goal))
        style, title = ("green", f"arrived at {self.node}") if arrived else ("red", f"stopped at {self.node}")
        panel = Panel(Group(grid_view(self.n, self.node, self.heading),
                            Text(result if not arrived else f"in {self.deliveries[-1][2]:.0f} s")),
                      title=f"[bold {style}]{title}[/]", border_style=style, expand=False)
        if self.live:                   # printed by the display thread, once the live panel is off
            self.live.end(panel, self.next_prompt())
        else:
            self.say(panel)
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
    EDGE_S seconds per edge, with the same calls as the real FSM: Mission.on_node at each node, and
    Mission.on_blocked when it meets one of its hidden obstacles (halfway along the edge: it turns back)."""

    def __init__(self, quiet=False):
        self.state, self.nav, self.reason, self.t_next = "IDLE", None, "start", 0.0
        self.obstacles = set()          # frozenset({(x, y), (x, y)}): hidden until the robot drives into one
        self.obstacle_ahead = False     # the next event is meeting an obstacle, not reaching a node
        self.quiet = quiet

    def step(self, cmd, t):
        from navigators import Mission
        if cmd == "STOP":
            self.state, self.reason = "IDLE", "STOP command"
        elif isinstance(cmd, tuple) and self.state == "IDLE":
            self.nav, self.state, self.t_next, self.obstacle_ahead = Mission(*cmd[1:]), "NODE", t, False
        if self.state != "NODE" or t < self.t_next:
            return
        nav = self.nav
        if self.obstacle_ahead:         # the sonar sees it a quarter of the way along: back to the node
            order, why = nav.on_blocked(nav.s_at + nav.g / 4, 0.30)
            self.obstacle_ahead, self.t_next = False, t + EDGE_S / 4
        else:                           # a node is reached (odometry: s_next)
            order, why = nav.on_node(None, nav.s_next if nav.node is not None else 0.0)
            if order == "STOP":
                self.state, self.reason = "IDLE", f"STOP at the node: {why}"
            elif frozenset((nav.node, nav.to)) in self.obstacles:
                self.obstacle_ahead, self.t_next = True, t + EDGE_S / 4
            else:
                self.t_next = t + EDGE_S
        if not self.quiet:
            console.print(f"[dim]  fake robot: {order.lower()}: {why}[/]")


def run_fake(cli, fsm=None, time_step=0.064):
    fsm = fsm or FakeFsm()
    cli.attach(fsm)
    while cli.running():                # the robot loop, 64 ms per cycle
        fsm.step(cli.command(), time.monotonic())
        time.sleep(time_step)


def ask_start(n, start=(0, 0), heading="E"):
    """Start node and heading typed by the user; Enter keeps the given ones."""
    while True:
        node = parse_node(Prompt.ask("Start node x y", default=f"{start[0]} {start[1]}"), n)
        if node is not None:
            break
        console.print(f"[red]type x y, each between 0 and {n - 1}[/]")
    heading = Prompt.ask("Heading", choices=list(HEADINGS), default=heading)
    return node, heading


def ask_speed(v0):
    """V0 (m/s) typed by the user; Enter keeps the given one."""
    while True:
        v = FloatPrompt.ask("V0, speed on straights (m/s)", default=v0)
        if v > 0:
            return v
        console.print("[red]V0 must be above 0[/]")


# ------------------------------------------------------------ settings for one run (params.ini is not changed)
def ini_value(path, section, key):
    """Value of key in [section] of a params .ini (a Python literal), or None if it is not there."""
    ini = configparser.ConfigParser(inline_comment_prefixes=("#",), interpolation=None)
    ini.optionxform = str
    ini.read(path)
    return ast.literal_eval(ini.get(section, key)) if ini.has_option(section, key) else None


def with_value(text, section, key, value):
    """The .ini text with `key = value` in [section]; everything else (comments, other sections) unchanged."""
    out, inside, done = [], False, False
    for line in text.splitlines(keepends=True):
        stripped = line.strip()
        if stripped.startswith("["):
            inside = stripped == f"[{section}]"
        elif inside and not done and re.match(rf"{re.escape(key)}\s*=", stripped):
            m = re.match(rf"(\s*{re.escape(key)}\s*=\s*)[^#\n]*?(\s*#.*)?(\n?)$", line)
            line, done = f"{m.group(1)}{value!r}{m.group(2) or ''}{m.group(3)}", True
        out.append(line)
    if not done:
        raise KeyError(f"[{section}] {key} not found")
    return "".join(out)


def run_params(path, values):
    """A temporary copy of the params file with this run's values {(section, key): value}. Returns its path
    (to delete at the end)."""
    with open(path) as f:
        text = f.read()
    for (section, key), value in values.items():
        text = with_value(text, section, key, value)
    fd, run_path = tempfile.mkstemp(prefix="params_run_", suffix=".ini")
    with os.fdopen(fd, "w") as f:
        f.write(text)
    return run_path


def start_args(argv, n):
    """--start x y (or x,y) and --heading E|N|W|S from the command line: (start or None, heading or None)."""
    start = heading = None
    if "--start" in argv:
        i = argv.index("--start")
        words = argv[i + 1:i + 3]
        start = parse_node(" ".join(words[:1]), n) or parse_node(" ".join(words), n)
        if start is None:
            raise SystemExit(f"--start: give the node as x y (or x,y), each between 0 and {n - 1}")
    if "--heading" in argv:
        i = argv.index("--heading")
        heading = argv[i + 1].upper() if i + 1 < len(argv) else ""
        if heading not in HEADINGS:
            raise SystemExit("--heading: one of E, N, W, S")
    return start, heading


def ask_run_settings(path, start_arg=None, heading_arg=None):
    """V0, start node and heading for this run, the file's values as defaults. Returns (v0, start, heading, n, g).
    start_arg / heading_arg (--start, --heading) are used as given, without asking."""
    v0, mission = ini_value(path, "line_follower", "V0"), ini_value(path, "fsm", "MISSION")
    if mission:                         # ("MISSION", start, heading, goal, n, g): the goal is not used
        start, heading, n, g = tuple(mission[1]), mission[2], mission[4], mission[5]
    else:
        start, heading, n, g = (0, 0), "E", DEFAULT_N, DEFAULT_G
    console.print(Panel(f"Settings for [bold]this run only[/]: {os.path.basename(path)} is not changed.\n"
                        "Enter keeps the value in brackets.", title="[bold]settings[/]", border_style="cyan",
                        expand=False))
    new_v0 = ask_speed(v0)
    if start_arg is None:
        start, heading = ask_start(n, start, heading_arg or heading)
    else:
        start, heading = start_arg, heading_arg or heading
        if heading_arg is None:
            heading = Prompt.ask("Heading", choices=list(HEADINGS), default=heading)
    console.print(f"V0 [bold]{new_v0}[/] m/s" + (f" (file: {v0})" if new_v0 != v0 else "")
                  + f", start [bold]{start}[/] facing [bold]{heading}[/], {n}x{n} grid, {g} m between nodes")
    return new_v0, start, heading, n, g


def start_cli(start, heading, n, g, cli=None, live=False, on_block=None):
    """Header, start view and the keyboard thread. Returns the DeliveryCli the robot loop polls
    (cli: an existing one, e.g. already wired to the hardware)."""
    console.print(Panel("[bold]ST5 Troller: deliveries on the grid[/]\n"
                        "x right, y up, (0, 0) bottom-left. Type a destination, Enter; q to quit."
                        + ("\nLive preview (--lp): path, obstacles and car redrawn while it drives." if live else ""),
                        border_style="cyan", expand=False))
    cli = cli or DeliveryCli(start, heading, n, g, live=live, on_block=on_block)
    console.print(Panel(grid_view(n, cli.node, cli.heading), title=f"[bold]start {cli.node}, facing {heading}[/]",
                        expand=False))
    threading.Thread(target=cli.input_loop, daemon=True).start()
    return cli


def main(argv=None):
    """python3 cli.py: the real robot (Pi). python3 cli.py --fake: on a PC, a fake robot (no camera, no serial).
    --lp (both): live preview, the path, the obstacles found and the car redrawn while the robot drives.
    --start x y / --heading N: where the robot is placed (else asked). --eight: laps on the figure 8 (cli_eight.py)."""
    argv = sys.argv[1:] if argv is None else argv
    if "--eight" in argv:
        import cli_eight
        return cli_eight.main(argv)
    live = "--lp" in argv
    start_arg, heading_arg = start_args(argv, DEFAULT_N)
    if "--fake" in argv:
        if start_arg is None:
            start, heading = ask_start(DEFAULT_N, heading=heading_arg or "E")
        else:
            start, heading = start_arg, heading_arg or Prompt.ask("Heading", choices=list(HEADINGS), default="E")
        fake = FakeFsm(quiet=live)      # with the live panel its lines would only push the panel down
        cli = start_cli(start, heading, DEFAULT_N, DEFAULT_G, live=live, on_block=fake.obstacles.add)
        try:
            run_fake(cli, fake)
        except KeyboardInterrupt:
            console.print("\nstopped (Ctrl+C)")
        cli.close()
        console.print(cli.summary())
        return
    import params                       # only its PATH: the robot modules are not loaded yet

    v0, start, heading, n, g = ask_run_settings(params.PATH, start_arg, heading_arg)
    # This run's copy of the params, with the new V0. Set before the robot modules load their values; the
    # environment variable also reaches the corner detector's process (a fresh interpreter: "spawn").
    run_path = run_params(params.PATH, {("line_follower", "V0"): v0})
    os.environ["ROBO_PARAMS"] = params.PATH = run_path
    try:
        import fsm                      # robot code (camera, serial): imported here, only on the Pi
        from robot_hardware import RobotHardware

        fsm.MISSION = ("MISSION", tuple(start), heading, tuple(start), n, g)   # not None: node detectors start
        fsm.AUTO_START = None           # no automatic mission: the CLI gives them
        cli = start_cli(start, heading, n, g, live=live)
        try:
            fsm.main(cli_hardware(RobotHardware, cli)(fsm.TIME_STEP))
        except KeyboardInterrupt:       # Ctrl+C still works (the motors stop on exit)
            console.print("\nstopped (Ctrl+C)")
        cli.close()
        console.print(cli.summary())
    finally:
        os.remove(run_path)


if __name__ == "__main__":
    main()
