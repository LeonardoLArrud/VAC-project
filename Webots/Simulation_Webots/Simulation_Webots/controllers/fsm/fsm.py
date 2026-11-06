"""Main loop of the basic FSM (v1). Same loop in Webots, on the real robot and offline.

Run in Webots: set the robot's controller to "fsm" (or, in the VM, ./run_extern.sh fsm).
It uses webots_hardware.py, where every hook is filled in (camera, motors, encoders,
rangefinder, servo, keyboard).
The offline test (offline/scenarios.py) calls main() with simulated hooks.

Writes fsm_log.csv in this folder, one row per cycle, for the timeline
(see offline/timeline.py).
"""
import csv
import json
import os

from fsm_basic import FSM, Params
from line_follower import LineFollower
from navigators import Explorer
from node_detector import NodeDetector

TIME_STEP = 64      # ms
AUTO_START = 1.0    # s; None = wait for a START command
LIVE_VIEW = True    # Webots only: http://localhost:8000/ shows the camera, the mask and the line points
# MISSION: sent at AUTO_START instead of START, and by key M. (start, heading, goal[, n, g]) on an n x n grid, g m apart.
# grid_5x5*.wbt: ("MISSION", (0, 0), "E", (4, 3)); experiments grid_*_extern.wbt: ("MISSION", (4, 1), "N", (6, 7), 9, 1.0)
MISSION = None
NAV = Explorer      # navigator factory: Explorer (the 8, START without a mission); tests: lambda: OrderList([...])


def main(hw, live_view=False, nav=NAV, detect=None):
    """hw: a hardware.Hardware subclass (WebotsHardware, RobotHardware, SimHardware).
    detect(reading) -> node_seen (d, exits) or None: a test's fake detector; default node_detector.py."""
    width, height, fov = hw.camera_info()
    follower = LineFollower(width, height, fov, TIME_STEP / 1000.0, hw.wheel_v_max())
    detector = NodeDetector(width, height, fov, follower.mask)
    view = None
    if live_view:
        from live_view import LiveView
        view = LiveView(follower)
    fsm = FSM(follower, Params(), nav=nav, on_change=lambda t, old, new, reason:
              print(f"[FSM] t={t:6.2f}s {old} -> {new} ({reason})"))
    if hasattr(hw, "attach"):                # e.g. cli_webots.py: watches the FSM to know when a delivery ends
        hw.attach(fsm)
    path = os.environ.get("FSM_LOG") or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "fsm_log.csv")
    view_path = os.environ.get("FSM_VIEW") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "mission.json")
    if os.path.exists(view_path):
        os.remove(view_path)                 # mission.json: what controllers/mission_view draws on the map
    started, shown = False, None
    with open(path, "w", newline="", buffering=1) as f:
        log = csv.writer(f)
        log.writerow(["t", "state", "v", "w", "dist", "line", "s", "node"])
        while hw.step():
            r = hw.read()
            if AUTO_START is not None and not started and r.t >= AUTO_START:
                started, r.cmd = True, r.cmd or MISSION or "START"
            r.node = detect(r) if detect else detector.step(r.image, r.s)
            if r.node:
                print(f"[NODE] t={r.t:6.2f}s s={r.s:.3f} d={r.node[0]:.3f} exits={sorted(r.node[1])}")
            follower.points = []
            v, w = fsm.step(r)
            hw.wheels(v, w)
            plan = json.dumps(fsm.nav.view()) if hasattr(fsm.nav, "view") else None
            if plan and plan != shown:
                with open(view_path + ".tmp", "w") as fv:
                    fv.write(plan)
                os.replace(view_path + ".tmp", view_path)   # atomic: mission_view never reads half a file
                shown = plan
            if view is not None:
                view.update(r.image, follower.points)
            log.writerow([f"{r.t:.3f}", fsm.state, f"{v:.4f}", f"{w:.4f}", f"{fsm.d:.3f}", fsm.line, f"{fsm.s:.3f}",
                          f"{r.node[0]:.3f} {''.join(sorted(r.node[1]))}" if r.node else ""])
    return fsm


if __name__ == "__main__":
    from webots_hardware import WebotsHardware   # Webots only; the real robot will use its own class
    main(WebotsHardware(TIME_STEP, mission=MISSION), live_view=LIVE_VIEW)
