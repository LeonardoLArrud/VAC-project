"""Runs fsm.main() (unchanged) with the simulated hooks of sim_hardware.py.

Usage:  python scenarios.py               all scenarios
        python scenarios.py straight_line  only those named

For each scenario it writes results/<name>.csv (fsm.py log) and
results/<name>_truth.csv (true pose and distance) and prints the verdict.
"""
import contextlib
import csv
import io
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(1, os.path.join(HERE, ".."))   # fsm.py, fsm_basic.py, hardware.py, line_follower.py

import fsm as ctrl  # noqa: E402
from sim_hardware import SimHardware  # noqa: E402
from world import Obstacle, World, arc, straight  # noqa: E402

OUT = os.path.join(HERE, "results")
os.makedirs(OUT, exist_ok=True)
SCENARIOS = {}
LINE = [straight((-0.5, 0.0), (4.5, 0.0))]


def scenario(fn):
    SCENARIOS[fn.__name__] = fn
    return fn


def run(w):
    os.environ["FSM_LOG"] = os.path.join(OUT, f"{w.name}.csv")
    with contextlib.redirect_stdout(io.StringIO()):
        f = ctrl.main(SimHardware(w))
    with open(os.path.join(OUT, f"{w.name}_truth.csv"), "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=list(w.log[0]))
        wr.writeheader()
        wr.writerows(w.log)
    with open(os.environ["FSM_LOG"]) as fh:
        log = list(csv.DictReader(fh))
    return f, log


def sequence(f):
    seq = [f.history[0][1]] if f.history else [f.state]
    seq += [new for _, old, new, _ in f.history if new != old]
    return seq


def t_of(f, new, after=0.0):
    return next((t for t, old, n, _ in f.history if n == new and n != old and t >= after), None)


def truth_at(w, t):
    return min(w.log, key=lambda r: abs(r["t"] - t))


# ------------------------------------------------------------------ scenarios
@scenario
def straight_line():
    w = World("straight_line", LINE, t_end=25)
    f, log = run(w)
    seq = sequence(f)
    e = max(abs(r["error"]) for r in w.log if r["t"] > 3)
    ok = seq == ["IDLE", "FOLLOW"] and e < 0.01
    return ok, f"{' > '.join(seq)}; max error {100 * e:.1f} cm; travelled {w.x:.2f} m"


@scenario
def curve():
    track = [straight((-0.5, 0), (0.6, 0)), arc((0.6, 0.5), 0.5, -math.pi / 2, math.pi / 2),
             straight((0.6, 1.0), (-1.5, 1.0))]
    w = World("curve", track, t_end=30)
    f, log = run(w)
    seq = sequence(f)
    e = max(abs(r["error"]) for r in w.log if r["t"] > 3)
    ok = seq == ["IDLE", "FOLLOW"] and e < 0.03 and w.x < 0.3
    return ok, f"{' > '.join(seq)}; R = 0.5 m; max error {100 * e:.1f} cm; left the curve: {w.x < 0.3}"


@scenario
def sharp_curve():
    track = [straight((-0.5, 0), (0.6, 0)), arc((0.6, 0.2), 0.2, -math.pi / 2, 0.0),
             straight((0.8, 0.2), (0.8, 4.0))]
    w = World("sharp_curve", track, t_end=25)
    f, log = run(w)
    seq = sequence(f)
    e = max(abs(r["error"]) for r in w.log if r["t"] > 18)
    ok = seq[-1] == "FOLLOW" and w.y > 1.0 and e < 0.01
    return ok, f"{' > '.join(seq)}; R = 0.2 m (< R_min); after the curve: y = {w.y:.2f} m, error {100 * e:.1f} cm"


@scenario
def static_obstacle():
    """Obstacle never leaves: WAIT, U-turn after 5 s, follow the line back."""
    line = [straight((-3.0, 0.0), (4.5, 0.0))]
    w = World("static_obstacle", line, [Obstacle(centre=(1.55, 0.0))], t_end=30)
    f, log = run(w)
    seq = sequence(f)
    t_wait, t_turn = t_of(f, "WAIT"), t_of(f, "TURN")
    r_wait = truth_at(w, t_wait + 0.3)
    moved = max(math.hypot(r["x"] - r_wait["x"], r["y"] - r_wait["y"])
                for r in w.log if t_wait + 0.3 < r["t"] < t_turn)
    d_stop = min(r["d_true"] for r in w.log if r["t"] < t_turn)
    end = w.log[-1]
    back = abs(abs(math.degrees(end["psi"])) - 180)
    ok = (seq == ["IDLE", "FOLLOW", "WAIT", "TURN", "FOLLOW"] and 0.18 <= d_stop <= 0.25 and moved < 0.005
          and abs(t_turn - t_wait - 5.0) < 0.1 and back < 10 and abs(end["error"]) < 0.015
          and end["x"] < r_wait["x"] - 0.5)
    return ok, (f"{' > '.join(seq)}; stopped {d_stop:.3f} m from the obstacle; U-turn {t_turn - t_wait:.1f} s "
                f"after stopping; turn took {t_of(f, 'FOLLOW', after=t_turn) - t_turn:.1f} s; at the end "
                f"heading {math.degrees(end['psi']):.0f} deg, error {100 * end['error']:.1f} cm, "
                f"{r_wait['x'] - end['x']:.2f} m back")


@scenario
def blocked_both_ways():
    """Obstacle ahead and another behind: one U-turn, then IDLE."""
    line = [straight((-3.0, 0.0), (4.5, 0.0))]
    w = World("blocked_both_ways", line, [Obstacle(centre=(1.55, 0.0)), Obstacle(centre=(-0.75, 0.0))], t_end=45)
    f, log = run(w)
    seq = sequence(f)
    ok = seq == ["IDLE", "FOLLOW", "WAIT", "TURN", "FOLLOW", "WAIT", "IDLE"] and f.reason == "blocked both ways"
    return ok, f"{' > '.join(seq)}; final reason: {f.reason}; stopped at t = {t_of(f, 'IDLE', after=2):.1f} s"


@scenario
def obstacle_leaves():
    w = World("obstacle_leaves", LINE, [Obstacle(t_on=5.0, t_off=10.0, ahead=0.45)], t_end=20)
    f, log = run(w)
    seq = sequence(f)
    t_back = t_of(f, "FOLLOW", after=10.0)
    delay = (t_back - 10.0) if t_back else math.nan
    d_min = min(r["d_true"] for r in w.log)
    ok = seq == ["IDLE", "FOLLOW", "WAIT", "FOLLOW"] and 1.0 <= delay <= 1.6 and d_min >= 0.15
    return ok, f"{' > '.join(seq)}; moved again {delay:.2f} s after the obstacle left; closest {d_min:.3f} m"


@scenario
def short_gap():
    track = [straight((-0.5, 0), (1.0, 0)), straight((1.22, 0), (4.5, 0))]
    w = World("short_gap", track, t_end=25, reference=LINE)
    f, log = run(w)
    seq = sequence(f)
    memory = sum(r["line"] == "MEMORY" for r in log)
    e = max(abs(r["error"]) for r in w.log if r["t"] > 3)
    ok = seq == ["IDLE", "FOLLOW"] and memory > 0 and e < 0.01
    return ok, f"{' > '.join(seq)}; 22 cm gap; {memory} frames in MEMORY; max error {100 * e:.1f} cm"


@scenario
def line_hidden_and_back():
    w = World("line_hidden_and_back", LINE, keys=[(6.0, "L"), (11.0, "L")], t_end=25)
    f, log = run(w)
    seq = sequence(f)
    end = w.log[-1]
    ok = (seq == ["IDLE", "FOLLOW", "SEARCH", "FOLLOW"] and abs(math.degrees(end["psi"])) < 10
          and abs(end["error"]) < 0.015)
    return ok, (f"{' > '.join(seq)}; camera blind from 6 to 11 s; at the end: heading "
                f"{math.degrees(end['psi']):+.1f} deg, error {100 * end['error']:.1f} cm")


@scenario
def end_of_line():
    w = World("end_of_line", [straight((-0.5, 0), (1.5, 0))], t_end=30)
    f, log = run(w)
    seq = sequence(f)
    past = w.x - 1.5
    ok = (seq == ["IDLE", "FOLLOW", "SEARCH", "IDLE"] and "exhausted" in f.reason
          and past <= 0.40 and abs(math.degrees(w.psi)) < 5)
    where = f"{100 * past:.0f} cm past the end" if past >= 0 else f"{-100 * past:.0f} cm before the end"
    return ok, f"{' > '.join(seq)}; stopped {where}; final heading {math.degrees(w.psi):+.1f} deg"


@scenario
def stop_command():
    w = World("stop_command", LINE, keys=[(6.0, "S"), (9.0, "G")], t_end=20)
    f, log = run(w)
    seq = sequence(f)
    t_stop = t_of(f, "IDLE", after=5.0)
    r0, r1 = truth_at(w, t_stop), truth_at(w, 9.0)
    slide = math.hypot(r1["x"] - r0["x"], r1["y"] - r0["y"])
    reaction = t_stop - 6.0
    ok = seq == ["IDLE", "FOLLOW", "IDLE", "FOLLOW"] and reaction <= 0.07 and slide < 0.01
    return ok, f"{' > '.join(seq)}; reaction {1000 * reaction:.0f} ms; slid {1000 * slide:.1f} mm before stopping"


@scenario
def stop_beats_obstacle():
    w = World("stop_beats_obstacle", LINE, [Obstacle(t_on=5.8, t_off=8.0, ahead=0.22)],
              keys=[(6.0, "S"), (11.0, "G")], t_end=20)
    f, log = run(w)
    seq = sequence(f)
    idle_8_11 = all(r["state"] == "IDLE" for r in log if 8.0 <= float(r["t"]) < 11.0)
    ok = seq[-1] == "FOLLOW" and idle_8_11 and "WAIT" in seq and seq[seq.index("WAIT") + 1] == "IDLE"
    return ok, f"{' > '.join(seq)}; obstacle left at 8 s and the robot stayed IDLE until G at 11 s: {idle_8_11}"


if __name__ == "__main__":
    names = sys.argv[1:] or list(SCENARIOS)
    failures = 0
    for name in names:
        ok, text = SCENARIOS[name]()
        failures += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {name:22s} {text}")
    print(f"\n{len(names) - failures}/{len(names)} scenarios passed")
    sys.exit(1 if failures else 0)
