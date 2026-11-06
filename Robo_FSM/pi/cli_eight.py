"""cli.py --eight: laps on the figure 8, chosen while the robot runs, with a live drawing of the 8.

Type a number of laps: the robot follows the line (the FSM's Explorer, straight through the crossing) and the CLI
counts the crossing passes (2 per lap), then sends STOP. The node detector is switched on for this mode only, so
the crossing is seen; between two crossings the car is placed on the drawing from the distance driven.
On the Pi: python3 cli.py --eight   On a PC: python3 cli.py --eight --fake   (o = fake obstacle while driving)
"""
import math
import os
import queue
import sys
import threading
import time

from rich.console import Group
from rich.panel import Panel
from rich.prompt import FloatPrompt, IntPrompt
from rich.table import Table
from rich.text import Text

from cli import CAR, PATH, QUIT, LivePreview, cli_hardware, console, ini_value, run_params, wait_for_q

W, H = 41, 11                       # the drawing of the 8, in characters
DEFAULT_LOOP_M = 3.0                # first guess of one loop's length (m), replaced by the first loop measured
UPDATE_S = 0.2                      # live panel: at most 5 redraws per second (the Pi 3's 64 ms loop comes first)
SETTINGS = [                        # (section, key, prompt): asked at launch on the Pi, for this run only
    ("line_follower", "V0", "V0, speed on straights (m/s)"),
    ("line_follower", "A_LAT", "A_LAT, curves: speed = sqrt(A_LAT x radius)"),
    ("fsm", "D_STOP", "D_STOP, stop for an obstacle closer than (m)"),
    ("fsm", "T_STATIC", "T_STATIC, still blocked this long: U-turn (s)"),
]
STATES = {"FOLLOW": "following the line", "NODE": "through the crossing", "WAIT": "obstacle: waiting",
          "TURN": "U-turn", "ALIGN": "stopping", "SEARCH": "line lost: searching", "CHECK": "checking the exit",
          "IDLE": "stopped"}


def eight_cells(w=W, h=H, samples=4000):
    """The 8 as character cells (col, row): (crossing, loop A, loop B), each loop in driving order from the
    crossing back to it. A lemniscate: only a picture, not the track's real shape."""
    def cell(t):
        d = 1 + math.sin(t) ** 2
        x, y = math.cos(t) / d, math.sin(t) * math.cos(t) / d    # x in [-1, 1], |y| <= 0.354
        return round((x + 1) / 2 * (w - 1)), round((0.354 - y) / 0.708 * (h - 1))

    crossing = cell(math.pi / 2)

    def loop(t0):
        cells = []
        for i in range(samples + 1):
            c = cell(t0 + math.pi * i / samples)
            if c != crossing and (not cells or cells[-1] != c):
                cells.append(c)
        return cells

    return crossing, loop(-math.pi / 2), loop(math.pi / 2)


CROSSING, LOOP_A, LOOP_B = eight_cells()


class EightPosition:
    """Where the car is on the 8: the loop (from the crossing passes), how far along it (from the distance driven
    since the last crossing or U-turn) and the direction (a U-turn reverses it)."""

    def __init__(self, loop_m=DEFAULT_LOOP_M):
        self.passes, self.loop, self.forward = 0, None, True     # loop: 0 / 1, None before the first crossing
        self.loop_m, self.measured = loop_m, False
        self.s_base, self.f_base, self.clean = 0.0, 0.0, False  # clean: no U-turn since the last crossing

    def crossing(self, s):
        if self.loop is not None and self.clean and s - self.s_base > 0.3:   # a whole loop, no U-turn: its length
            self.loop_m, self.measured = s - self.s_base, True
        self.passes += 1
        self.loop = 0 if self.loop is None else 1 - self.loop   # straight through the crossing: the other loop
        self.s_base, self.f_base, self.clean = s, (0.0 if self.forward else 1.0), True

    def u_turn(self, s):
        self.f_base, self.s_base = self.fraction(s) or 0.0, s
        self.forward, self.clean = not self.forward, False

    def fraction(self, s):              # 0 at the crossing, 1 back at it; None before the first crossing
        if self.loop is None:
            return None
        f = self.f_base + (1 if self.forward else -1) * (s - self.s_base) / self.loop_m
        return min(0.97, max(0.03, f))


def eight_view(loop, fraction):
    """The 8 with the car on it: the current loop highlighted."""
    cells = (LOOP_A, LOOP_B)
    car = None if loop is None else cells[loop][min(len(cells[loop]) - 1, int(fraction * len(cells[loop])))]
    lines = []
    for row in range(H):
        line = Text()
        for col in range(W):
            c = (col, row)
            if c == car:
                line.append("●", style=CAR)
            elif c == CROSSING:
                line.append("✕", style="bold")
            elif loop is not None and c in cells[loop]:
                line.append("•", style=PATH)
            elif c in LOOP_A or c in LOOP_B:
                line.append("·", style="dim")
            else:
                line.append(" ")
        lines.append(line)
    return Group(*lines)


def eight_panel(snap):
    loop, fraction, forward, passes, target, state, v, d, u_turns, measured, loop_m, secs, dist = snap
    laps = f"lap {passes // 2 + 1}" + (f" of {target // 2}" if target else " (until q)")
    if passes >= (target or math.inf):
        laps = f"{target // 2} lap{'s' * (target != 2)} done: stopping"
    where = ("position: after the first crossing" if loop is None else
             f"loop {'AB'[loop]}, {'forward' if forward else 'reversed (U-turn)'} · loop ≈ {loop_m:.1f} m"
             + ("" if measured else " (guess)"))
    what = STATES.get(state, state)
    if state == "WAIT" and math.isfinite(d):
        what = f"obstacle at {d:.2f} m: waiting"
    lines = [Text.from_markup(f"[bold]{laps}[/] · crossing passes {passes} · {dist:.1f} m · {secs:.0f} s"),
             Text(f"{v:.2f} m/s · {what} · U-turns {u_turns}"), Text(where, style="dim")]
    return Panel(Group(eight_view(loop, fraction or 0.0), *lines), title="[bold]figure 8[/]", subtitle="q: stop",
                 expand=False)


class EightCli:
    """Between the keyboard (input thread) and the robot loop (command(), every cycle), like cli.DeliveryCli:
    a run = a number of laps (0 = until q). Laps are counted at the crossing: 2 passes per lap."""

    def __init__(self, ask=console.input, say=console.print, live=True, on_obstacle=None):
        self.pos = EightPosition()
        self.ask, self.say = ask, say
        self.live = LivePreview() if live else None
        self.on_obstacle = on_obstacle  # --fake: o (while driving) puts an obstacle in front of the fake robot
        self.fsm, self.pending, self.quit = None, queue.Queue(), False
        self.active = self.sent = self.stopping = False
        self.target, self.passes0, self.u_turns, self.t0, self.s0 = None, 0, 0, 0.0, None
        self.seen, self.last_key, self.t_drawn = 0, None, 0.0    # FSM history entries already read; redraws
        self.runs = []                  # (laps asked or None, laps done, U-turns, seconds, result)

    def attach(self, fsm):
        self.fsm = fsm

    def running(self):
        return not self.quit

    def busy(self):
        return self.active

    # ------------------------------------------------------------ keyboard side
    def handle(self, text):
        text = text.strip().lower()
        if text == QUIT:
            if self.active:
                self.record("stopped (q)")
            self.quit = True
            self.pending.put("STOP")
            return False
        if self.active:
            self.say("[yellow]driving: q stops[/]")
            return True
        try:
            laps = int(text)
        except ValueError:
            laps = -1
        if laps < 0:
            self.say("[red]type a number of laps (0 = until q), or q to quit[/]")
            return True
        self.active, self.sent, self.stopping = True, False, False
        self.target, self.passes0, self.u_turns, self.t0 = 2 * laps or None, self.pos.passes, 0, time.monotonic()
        self.s0 = None                  # distance at the start of this run: read from the FSM once it is driving
        self.last_key = None
        if self.live:
            self.live.start(eight_panel, f"{laps or 'until q'} lap{'s' * (laps != 1)}: starting...")
        else:
            self.say(f"{laps or 'until q'} lap{'s' * (laps != 1)}: starting (q + Enter stops)")
        self.pending.put("START")
        return True

    def key(self, k):                   # single keys while driving (wait_for_q)
        if k == "o" and self.on_obstacle:
            self.on_obstacle()

    def input_loop(self):
        self.say(self.next_prompt())
        while not self.quit:
            if self.live:
                self.live.idle.wait()
            try:
                text = self.ask("> ")
            except (EOFError, KeyboardInterrupt):
                text = QUIT
            if not self.handle(text):
                break
            if self.live and self.busy():
                wait_for_q(self)

    def next_prompt(self):
        fake = ", o while driving: fake obstacle" if self.on_obstacle else ""
        return f"[bold cyan]Laps to drive (0 = until q), q to quit{fake}[/]"

    def close(self):
        if self.live:
            self.live.close()

    # ------------------------------------------------------------ robot side, every cycle
    def command(self):
        self.follow_fsm()
        try:
            cmd = self.pending.get_nowait()
        except queue.Empty:
            return None
        if cmd == "START":
            self.sent = True
        return cmd

    def follow_fsm(self):
        fsm = self.fsm
        if fsm is None:
            return
        for _, old, new, _ in fsm.history[self.seen:]:          # transitions since the last cycle
            if new == "NODE":
                self.pos.crossing(fsm.s)
            elif new == "TURN" and old == "WAIT":                 # obstacle still there: U-turn
                self.pos.u_turn(fsm.s)
                self.u_turns += 1
        self.seen = len(fsm.history)
        if not (self.active and self.sent):
            return
        done = self.pos.passes - self.passes0
        if self.target and done >= self.target and not self.stopping:
            self.stopping = True
            self.pending.put("STOP")                              # the laps are done: stop at this crossing
        if fsm.state == "IDLE":
            self.record("done" if self.stopping else fsm.reason)
        elif self.live:
            self.draw(fsm, done)

    def draw(self, fsm, done):
        """A snapshot for the live panel when something shown changed: at once for a crossing (lap, loop),
        else at most every UPDATE_S (speed and distance change every cycle)."""
        if self.s0 is None:
            self.s0 = fsm.s
        f, dist = self.pos.fraction(fsm.s), fsm.s - self.s0
        key = (done, self.pos.loop, round(f or 0, 2), fsm.state, self.u_turns, round(fsm.v, 2), round(dist, 1))
        now = time.monotonic()
        if key != self.last_key and (now - self.t_drawn >= UPDATE_S or key[:2] != (self.last_key or (None,) * 2)[:2]):
            self.last_key, self.t_drawn = key, now
            self.live.update((self.pos.loop, f, self.pos.forward, done, self.target, fsm.state, fsm.v, fsm.d,
                              self.u_turns, self.pos.measured, self.pos.loop_m, now - self.t0, dist))

    def record(self, result):
        secs = time.monotonic() - self.t0
        laps_done = (self.pos.passes - self.passes0) // 2
        self.runs.append((self.target // 2 if self.target else None, laps_done, self.u_turns, secs, result))
        self.active = self.sent = False
        ok = result == "done"
        msg = Text.from_markup(f"[bold {'green' if ok else 'red'}]{laps_done} lap{'s' * (laps_done != 1)} in "
                               f"{secs:.0f} s[/]" + ("" if ok else f" · {result}"))
        if self.live:
            self.live.end(msg, self.next_prompt())
        elif not self.quit:
            self.say(msg)
            self.say(self.next_prompt())

    def summary(self):
        table = Table(title="Runs on the 8")
        for col in ("#", "laps asked", "laps done", "U-turns", "time", "result"):
            table.add_column(col)
        for i, (asked, done, u_turns, secs, result) in enumerate(self.runs, 1):
            table.add_row(str(i), str(asked or "until q"), str(done), str(u_turns), f"{secs:.0f} s", result,
                          style="green" if result == "done" else "red")
        return table


class FakeEightFsm:
    """--eight --fake: the FSM's part on the 8 without camera or wheels. s grows at v; a crossing every LOOP_M
    (state NODE for one cycle); an obstacle (o) stops it (WAIT), then after t_static a U-turn (TURN): the history
    and the states are the ones the CLI reads on the real FSM."""

    LOOP_M = 2.4

    def __init__(self, v=0.3, t_static=2.0):
        self.state, self.reason, self.history = "IDLE", "start", []
        self.s, self.v_set, self.v, self.d, self.t_static = 0.0, v, 0.0, math.inf, t_static
        self.to_cross, self.blocked, self.t = self.LOOP_M / 2, False, 0.0

    def go(self, new, reason):
        self.history.append((self.t, self.state, new, reason))
        self.state, self.reason, self.t_enter = new, reason, self.t

    def obstacle(self):
        self.blocked = True

    def step(self, cmd, t, dt):
        self.t = t
        if cmd == "STOP" and self.state != "IDLE":
            self.go("IDLE", "STOP command")
        elif cmd == "START" and self.state == "IDLE":
            self.go("FOLLOW", "START command")
        self.v, self.d = 0.0, math.inf
        if self.state == "NODE":
            self.go("FOLLOW", "bar passed")
        elif self.state == "TURN":
            self.to_cross = self.LOOP_M - self.to_cross               # back the way it came
            self.go("FOLLOW", "turn done")
        elif self.state == "WAIT":
            self.d = 0.20
            if t - self.t_enter >= self.t_static:
                self.blocked = False
                self.go("TURN", f"still blocked after {self.t_static:.0f} s: U-turn")
        elif self.state == "FOLLOW":
            if self.blocked:
                self.d = 0.20
                self.go("WAIT", "obstacle at 0.20 m")
                return
            self.v = self.v_set
            self.s += self.v * dt
            self.to_cross -= self.v * dt
            if self.to_cross <= 0:
                self.to_cross += self.LOOP_M
                self.go("NODE", "node at 0.40 m, exits LRS: STRAIGHT (explorer)")


def run_fake(cli, fsm, time_step=0.064):
    cli.attach(fsm)
    while cli.running():
        fsm.step(cli.command(), time.monotonic(), time_step)
        time.sleep(time_step)


def ask_settings(path):
    """V0, A_LAT, D_STOP and T_STATIC for this run, params file values as defaults. Returns {(section, key): v}."""
    console.print(Panel(f"Settings for [bold]this run only[/]: {os.path.basename(path)} is not changed.\n"
                        "Enter keeps the value in brackets.", title="[bold]the 8: settings[/]", border_style="cyan",
                        expand=False))
    values = {}
    for section, key, prompt in SETTINGS:
        old = ini_value(path, section, key)
        while True:
            new = FloatPrompt.ask(prompt, default=old)
            if new > 0:
                break
            console.print(f"[red]{key} must be above 0[/]")
        values[(section, key)] = new
    console.print(" · ".join(f"{k} [bold]{v}[/]" for (_, k), v in values.items()))
    return values


def start(cli):
    console.print(Panel("[bold]ST5 Troller: laps on the 8[/]\nType a number of laps, Enter. Laps are counted at the "
                        "crossing (2 passes per lap); the car on the drawing is placed from the distance driven.",
                        border_style="cyan", expand=False))
    threading.Thread(target=cli.input_loop, daemon=True).start()


def main(argv):
    if "--fake" in argv:
        v0 = FloatPrompt.ask("V0 of the fake robot (m/s)", default=0.3)
        fake = FakeEightFsm(v=v0)
        cli = EightCli(on_obstacle=fake.obstacle)
        start(cli)
        try:
            run_fake(cli, fake)
        except KeyboardInterrupt:
            console.print("\nstopped (Ctrl+C)")
        cli.close()
        console.print(cli.summary())
        return
    import params                       # only its PATH: the robot modules are not loaded yet

    run_path = run_params(params.PATH, ask_settings(params.PATH))
    os.environ["ROBO_PARAMS"] = params.PATH = run_path   # before the robot modules load (and the spawned detector)
    try:
        import fsm                      # robot code (camera, serial): only on the Pi
        from robot_hardware import RobotHardware

        if fsm.MISSION is None:         # not None: fsm.main starts the node detector, which sees the crossing.
            fsm.MISSION = ("EIGHT",)    # (Only its None-ness is used: AUTO_START is off, no mission is sent.)
        fsm.AUTO_START = None           # START comes from the CLI
        cli = EightCli()
        start(cli)
        try:
            fsm.main(cli_hardware(RobotHardware, cli)(fsm.TIME_STEP))
        except KeyboardInterrupt:
            console.print("\nstopped (Ctrl+C)")
        cli.close()
        console.print(cli.summary())
    finally:
        os.remove(run_path)


if __name__ == "__main__":
    main(sys.argv[1:])
