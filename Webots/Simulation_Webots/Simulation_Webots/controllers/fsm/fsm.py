"""Main loop of the basic FSM (v1). Same loop in Webots, on the real robot and offline.

Run in Webots: set the robot's controller to "fsm" (or, in the VM, ./run_extern.sh fsm).
It uses webots_hardware.py, where every hook is filled in (camera, motors, encoders,
rangefinder, servo, keyboard).
The offline test (offline/scenarios.py) calls main() with simulated hooks.

Writes fsm_log.csv in this folder, one row per cycle, for the timeline
(see offline/timeline.py).
"""
import csv
import os

from fsm_basic import FSM, Params
from line_follower import LineFollower

TIME_STEP = 64      # ms
AUTO_START = 1.0    # s; None = wait for a START command


def main(hw):
    """hw: a hardware.Hardware subclass (WebotsHardware, RobotHardware, SimHardware)."""
    width, height, fov = hw.camera_info()
    follower = LineFollower(width, height, fov, TIME_STEP / 1000.0, hw.wheel_v_max())
    fsm = FSM(follower, Params(), on_change=lambda t, old, new, reason:
              print(f"[FSM] t={t:6.2f}s {old} -> {new} ({reason})"))
    path = os.environ.get("FSM_LOG") or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "fsm_log.csv")
    started = False
    with open(path, "w", newline="", buffering=1) as f:
        log = csv.writer(f)
        log.writerow(["t", "state", "v", "w", "dist", "line"])
        while hw.step():
            r = hw.read()
            if AUTO_START is not None and not started and r.t >= AUTO_START:
                started, r.cmd = True, r.cmd or "START"
            v, w = fsm.step(r)
            hw.wheels(v, w)
            log.writerow([f"{r.t:.3f}", fsm.state, f"{v:.4f}", f"{w:.4f}", f"{fsm.d:.3f}", fsm.line])
    return fsm


if __name__ == "__main__":
    from webots_hardware import WebotsHardware   # Webots only; the real robot will use its own class
    main(WebotsHardware(TIME_STEP))
