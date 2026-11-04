"""Runs fsm.main() (unchanged) with the simulated hooks of sim_hardware.py.

Usage:  python scenarios.py               all scenarios
        python scenarios.py straight_line  only those named

For each scenario it writes results/<name>.csv (fsm.py log) and
results/<name>_truth.csv (true pose and distance) and prints the verdict.
"""
import contextlib
import csv
import io
import itertools
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(1, os.path.join(HERE, ".."))   # fsm.py, fsm_basic.py, hardware.py, line_follower.py

import fsm as ctrl  # noqa: E402

ctrl.MISSION = None   # offline, a mission comes from the scenario keys, never from the Webots config
from sim_hardware import SimHardware  # noqa: E402
from navigators import OrderList  # noqa: E402
from world import RES, Obstacle, World, arc, eight_track, grid_track, straight  # noqa: E402

OUT = os.path.join(HERE, "results")
os.makedirs(OUT, exist_ok=True)
SCENARIOS = {}
LINE = [straight((-0.5, 0.0), (4.5, 0.0))]


def scenario(fn):
    SCENARIOS[fn.__name__] = fn
    return fn


def run(w, nav=None, detect=None):
    os.environ["FSM_LOG"] = os.path.join(OUT, f"{w.name}.csv")
    os.environ["FSM_VIEW"] = os.path.join(OUT, f"{w.name}_mission.json")
    with contextlib.redirect_stdout(io.StringIO()) as printed:
        f = ctrl.main(SimHardware(w), nav=nav or ctrl.NAV, detect=detect)
    f.printed = printed.getvalue()
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
    """Obstacle ahead and another behind: it never gives up, a U-turn each time it is blocked (tries again)."""
    line = [straight((-3.0, 0.0), (4.5, 0.0))]
    w = World("blocked_both_ways", line, [Obstacle(centre=(1.55, 0.0)), Obstacle(centre=(-0.75, 0.0))], t_end=45)
    f, log = run(w)
    seq = sequence(f)
    turns = seq.count("TURN")
    ok = seq[:7] == ["IDLE", "FOLLOW", "WAIT", "TURN", "FOLLOW", "WAIT", "TURN"] and "IDLE" not in seq[1:]
    return ok, f"{' > '.join(seq)}; {turns} U-turns, never IDLE; last reason: {f.reason}"


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


# ------------------------------------------------------------------ real node detector (PDF step 2)
G = 0.50   # m between grid nodes


def events(log):
    """node_seen events of a run, from the log: (t, s, d, exits)."""
    return [(float(r["t"]), float(r["s"]), float(r["node"].split()[0]), set(r["node"].split()[1]))
            for r in log if r["node"]]


def five_seeds(fn):
    """Scenario that passes only if fn(seed) passes for the 5 noise seeds 0..4."""
    def run_all():
        res = [fn(seed) for seed in range(5)]
        bad = [i for i, (ok, _) in enumerate(res) if not ok]
        return not bad, f"5/5 seeds; seed 0: {res[0][1]}" if not bad else f"failed seeds {bad}; seed {bad[0]}: {res[bad[0]][1]}"
    run_all.__name__ = fn.__name__
    return scenario(run_all)


def d_error(w, ev, nodes, axis):
    """Largest |d - true distance to the node centre| over the events (axis 0: x, 1: y)."""
    return max(abs(d - (n - truth_at(w, t)["xy"[axis]])) for (t, s, d, e), n in zip(ev, nodes))


def reflections(w, seed, extra=(), n=600):
    """Floor reflections: white spots of 4 and 8 mm, 1.5 to 6 cm from the lines, plus 8 mm ones at `extra`."""
    rng = np.random.default_rng(seed)
    pts = np.vstack(w.segments)
    spots = [(*(pts[rng.integers(len(pts))] + rng.uniform(0.015, 0.06) * np.array([math.cos(a), math.sin(a)])),
              rng.integers(1, 3)) for a in rng.uniform(0, 2 * math.pi, n)]
    for x, y, k in spots + [(x, y, 2) for x, y in extra]:
        i, j = int((y - w.y0) / RES), int((x - w.x0) / RES)
        w.floor[i : i + k, j : j + k] = 230


def row_east(name, seed, spots=False):
    w = World(f"{name}_{seed}", grid_track(), pose0=(0.0, 2 * G, 0.0), seed=seed, t_end=30)
    if spots:                                        # and 4 on the axis beyond the T at (4, 2), where an S would be
        reflections(w, seed, extra=[(4 * G + 0.05 + 0.03 * k, 2 * G) for k in range(4)])
    f, log = run(w)
    ev = events(log)
    ex = [e for *_, e in ev]
    err = d_error(w, ev, [G, 2 * G, 3 * G, 4 * G], 0) if len(ev) == 4 else math.inf
    ok = ex == [{"L", "S", "R"}] * 3 + [{"L", "R"}] and f.state == "IDLE" and "no single exit" in f.reason and err < 0.02
    return ok, f"exits {[''.join(sorted(e)) for e in ex]}; d error {100 * err:.1f} cm; final {f.reason!r}"


@five_seeds
def grid_row(seed):
    return row_east("grid_row", seed)


@five_seeds
def grid_reflections(seed):
    return row_east("grid_reflections", seed, spots=True)


@five_seeds
def grid_column(seed):
    w = World(f"grid_column_{seed}", grid_track(), pose0=(4 * G, 0.0, math.pi / 2), seed=seed, t_end=26)
    f, log = run(w)
    ev = events(log)[:4]
    ex = [e for *_, e in ev]
    err = d_error(w, ev, [G, 2 * G, 3 * G, 4 * G], 1) if len(ev) == 4 else math.inf
    ok = ex == [{"L", "S"}] * 3 + [{"L"}] and err < 0.02
    return ok, f"first exits {[''.join(sorted(e)) for e in ex]}; d error {100 * err:.1f} cm"


def eight(seed, detect=None):
    w = World(f"eight_laps_{seed}" if detect is None else "eight_laps_v1", eight_track(), pose0=(-0.45, -0.45, math.pi / 4),
              seed=seed, t_end=260)
    f, log = run(w, detect=detect)
    return w, f, log, max(abs(r["error"]) for r in w.log if r["t"] > 3)


EIGHT_V1 = []


@five_seeds
def eight_laps(seed):
    if not EIGHT_V1:
        EIGHT_V1.append(eight(0, lambda r: None)[3])
    w, f, log, e = eight(seed)
    ev = [x for x in events(log) if x[0] < w.t_end - 5]   # an event comes ~4 s before its crossing
    near = [math.hypot(r["x"], r["y"]) < 0.05 for r in w.log]
    passes = sum(b and not a for a, b in itertools.pairwise(near))
    states = {r["state"] for r in log if float(r["t"]) > 1.1}
    ok = (passes >= 6 and len(ev) == passes and all(x == {"L", "S", "R"} for *_, x in ev)
          and states == {"FOLLOW", "NODE"} and e <= EIGHT_V1[0] + 0.005)
    return ok, (f"{passes} crossings, {len(ev)} events {sorted({''.join(sorted(x)) for *_, x in ev})}; states {sorted(states)}; "
                f"max error {100 * e:.1f} cm (v1 {100 * EIGHT_V1[0]:.1f} cm)")


@five_seeds
def order_list_cell_real(seed):
    cs = [(0.5, 0.0), (0.5, 0.5), (0.0, 0.5), (0.0, 0.0)]
    w = World(f"order_list_cell_real_{seed}", [straight(cs[i - 1], cs[i]) for i in range(4)], pose0=(0.05, 0.0, 0.0),
              seed=seed, t_end=45)
    f, log = run(w, nav=lambda: OrderList(["LEFT"] * 4))
    seq, ev, end = sequence(f), events(log), w.log[-1]
    err, turned = math.hypot(end["x"] - 0.5, end["y"]), math.degrees(end["psi"])
    ok = (seq == ["IDLE", "FOLLOW"] + ["NODE", "TURN", "FOLLOW"] * 4 + ["NODE", "IDLE"] and err < 0.03
          and abs(turned - 360) < 10 and [e for *_, e in ev] == [{"L"}] * 5)
    return ok, f"{len(ev)} events {sorted({''.join(sorted(x)) for *_, x in ev})}; {100 * err:.1f} cm from the first node; heading {turned:+.0f} deg"


@five_seeds
def loose_tape(seed):
    track = LINE + [straight((1.5, 0.035), (1.5, 0.085)), straight((2.5, -0.04), (2.55, -0.04))]
    w = World(f"loose_tape_{seed}", track, seed=seed, t_end=25, reference=LINE)
    f, log = run(w)
    ev = events(log)
    ok = sequence(f) == ["IDLE", "FOLLOW"] and not ev
    return ok, f"{' > '.join(sequence(f))}; {len(ev)} events; tape pieces across and along the line"


@five_seeds
def mission_grid(seed):
    """MISSION((0, 0), E, (4, 3)) on the 5x5 grid with the real detector: 7 edges, stop on (4, 3)."""
    w = World(f"mission_grid_{seed}", grid_track(), seed=seed, t_end=70, keys=[(0.5, ("MISSION", (0, 0), "E", (4, 3)))])
    f, log = run(w)
    edges = sum(old == "FOLLOW" and new == "NODE" for _, old, new, _ in f.history)
    err = math.hypot(w.x - 4 * G, w.y - 3 * G)
    ok = f.state == "IDLE" and "arrived" in f.reason and edges == 7 and err < 0.03
    turns = [why.split(": ")[-1] for _, old, new, why in f.history if new == "TURN"]
    return ok, f"{edges} edges, turns {turns}; final {f.reason!r}; {100 * err:.1f} cm from (4, 3)"


def mission(name, seed, segments=None, obstacles=(), heading="E"):
    w = World(f"{name}_{seed}", segments or grid_track(), obstacles, seed=seed, t_end=110,
              keys=[(0.5, ("MISSION", (0, 0), heading, (4, 3)))])
    f, log = run(w)
    err = math.hypot(w.x - 4 * G, w.y - 3 * G)
    edges = sum(old == "FOLLOW" and new == "NODE" for _, old, new, _ in f.history)
    return w, f, f.printed, err, edges


@five_seeds
def mission_blocked(seed):
    """Box in the middle of edge (2, 3)-(3, 3): WAIT, U-turn, replan around it, arrive."""
    w, f, out, err, edges = mission("mission_blocked", seed, obstacles=[Obstacle(centre=(2.5 * G, 3 * G))])
    blocked = [why for _, _, new, why in f.history if new == "TURN" and "blocked" in why]
    d_min = min(r["d_true"] for r in w.log)
    ok = "arrived" in f.reason and len(blocked) == 1 and "(2, 3)-(3, 3)" in blocked[0] and err < 0.03 and d_min > 0.15
    return ok, f"{blocked}; final {f.reason!r}; {100 * err:.1f} cm from (4, 3); closest to the box {d_min:.2f} m"


@five_seeds
def mission_missed_node(seed):
    """The bar of (0, 2) is missing: node missed, counted and logged; the mission still arrives."""
    segs = grid_track()
    segs[2] = straight((0.12, 2 * G), (4 * G, 2 * G))   # row j = 2 starts 12 cm east of (0, 2)
    w, f, out, err, edges = mission("mission_missed_node", seed, segs)
    ok = "arrived" in f.reason and edges == 6 and "node (0, 2) missed" in out and err < 0.03
    return ok, f"{edges} nodes seen; missed logged: {'node (0, 2) missed' in out}; final {f.reason!r}; {100 * err:.1f} cm"


@five_seeds
def mission_wrong_heading(seed):
    """MISSION says N but the robot faces E: the exits seen do not fit the map, pose lost."""
    w, f, out, err, edges = mission("mission_wrong_heading", seed, heading="N")
    ok = f.state == "IDLE" and "pose lost" in f.reason and edges == 1
    return ok, f"final {f.reason!r} after {edges} node"


@five_seeds
def mission_missing_edge(seed):
    """No tape between (2, 3) and (3, 3): STRAIGHT is not among the exits seen, the edge leaves the map, replan."""
    segs = grid_track()
    segs[3:4] = [straight((0, 3 * G), (2 * G, 3 * G)), straight((3 * G, 3 * G), (4 * G, 3 * G))]
    w, f, out, err, edges = mission("mission_missing_edge", seed, segs)
    at = [why for _, _, new, why in f.history if new == "NODE" and "at (2, 3)" in why]
    ok = "arrived" in f.reason and err < 0.03 and len(at) == 1 and "exits LR" in at[0] and "STRAIGHT" not in at[0]
    return ok, f"at (2, 3): {at}; final {f.reason!r}; {100 * err:.1f} cm from (4, 3)"


if __name__ == "__main__":
    names = sys.argv[1:] or list(SCENARIOS)
    failures = 0
    for name in names:
        ok, text = SCENARIOS[name]()
        failures += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {name:22s} {text}")
    print(f"\n{len(names) - failures}/{len(names)} scenarios passed")
    sys.exit(1 if failures else 0)
