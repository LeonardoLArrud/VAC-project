"""FSM of the ST5 robot (IDLE, FOLLOW, WAIT, SEARCH, TURN) and its main loop. Writes fsm_log.csv."""
import csv
import math
import os
from collections import deque

from line_follower import LOST, OK, V0, K, LineFollower

TIME_STEP = 64
AUTO_START = 1.0

D_STOP, D_SLOW = 0.25, 0.60
SLOW_FRACTION = 0.25
T_STATIC, T_MIN = 2.0, 0.3
N_OK = 3
SEARCH_RATE = TURN_RATE = 1.0
TURN_MIN, TURN_MAX = math.radians(100), math.radians(420)
TURN_TOL_Y = 0.04

IDLE, FOLLOW, WAIT, SEARCH, TURN = "IDLE", "FOLLOW", "WAIT", "SEARCH", "TURN"


class FSM:
    def __init__(self, follower):
        self.follower = follower
        self.state, self.reason, self.history = IDLE, "start", []
        self.t = self.t_enter = self.heading = self.heading0 = 0.0
        self.v = self.w = 0.0
        self.d, self.dists = float("inf"), deque(maxlen=3)
        self.line, self.side = LOST, 1.0
        self.n_ok = 0

    def step(self, t, image, dist, heading, cmd):
        self.t, self.heading = t, heading
        self.dists.append(dist)
        self.d = sorted(self.dists)[len(self.dists) // 2]
        if cmd == "STOP" and self.state != IDLE:
            self.go(IDLE, "STOP command")
        elif self.state == FOLLOW and self.d < D_STOP:
            self.go(WAIT, f"obstacle at {self.d:.2f} m")
        self.v, self.w = getattr(self, self.state.lower())(image, cmd)
        return self.v, self.w

    def go(self, new, reason):
        print(f"[FSM] t={self.t:6.2f}s {self.state} -> {new} ({reason})")
        self.history.append((self.t, self.state, new, reason))
        self.state, self.reason = new, reason
        self.t_enter, self.heading0, self.n_ok = self.t, self.heading, 0

    def look(self, image):
        out = self.follower.step(image, 0.0, 0.0, self.w)
        self.line = out.status
        return out

    def idle(self, image, cmd):
        if cmd == "START":
            self.go(FOLLOW, "START command")
        return 0.0, 0.0

    def follow(self, image, cmd):
        v_max = V0 * min(1.0, max(SLOW_FRACTION, (self.d - D_STOP) / (D_SLOW - D_STOP)))
        out = self.follower.step(image, v_max, self.v, self.w)
        self.line, self.side = out.status, out.side
        if out.status == LOST and self.t - self.t_enter >= T_MIN:
            self.go(SEARCH, "line lost")
            return 0.0, 0.0
        return out.v, out.w

    def wait(self, image, cmd):
        if self.d > D_STOP:
            self.go(FOLLOW, "obstacle left: not static")
        elif self.t - self.t_enter >= T_STATIC:
            self.go(TURN, f"still blocked after {T_STATIC:.0f} s: U-turn")
        return 0.0, 0.0

    def search(self, image, cmd):
        if self.look(image).status == OK:
            self.n_ok += 1
            if self.n_ok >= N_OK and self.t - self.t_enter >= T_MIN:
                self.go(FOLLOW, "line found")
            return 0.0, 0.0
        self.n_ok = 0
        if abs(self.heading - self.heading0) >= 2 * math.pi:
            self.go(IDLE, "line lost (search exhausted)")
            return 0.0, 0.0
        return 0.0, self.side * SEARCH_RATE

    def turn(self, image, cmd):
        """Spin left until the line is ahead and centred: at least TURN_MIN (so it does not stop on the
        line it came from), at most TURN_MAX (over 360 deg on purpose: the wheels slip)."""
        turned = abs(self.heading - self.heading0)
        if turned >= TURN_MIN:
            target = self.follower.find_target(image)
            self.line = LOST if target is None else OK
            if target is not None and abs(target[1]) <= TURN_TOL_Y:
                self.end_turn(FOLLOW, f"turn done after {math.degrees(turned):.0f} deg, line ahead")
                return 0.0, 0.0
        if turned >= TURN_MAX:
            self.end_turn(SEARCH, f"turned {math.degrees(turned):.0f} deg without the line")
            return 0.0, 0.0
        return 0.0, TURN_RATE

    def end_turn(self, new, reason):
        self.dists.clear()
        self.follower.reset()
        self.go(new, reason)


def main(hw):
    width, height, fov = hw.camera_info()
    fsm = FSM(LineFollower(width, height, fov, TIME_STEP / 1000.0, hw.wheel_v_max()))
    path = os.environ.get("FSM_LOG") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "fsm_log.csv")
    with open(path, "w", newline="", buffering=1) as f:
        log = csv.writer(f)
        log.writerow(["t", "state", "v", "w", "dist", "line"])
        while hw.step():
            t, cmd = hw.time(), hw.command()
            if AUTO_START is not None and not fsm.history and t >= AUTO_START:
                cmd = cmd or "START"
            fsm.follower.points = []
            v, w = fsm.step(t, hw.camera_image(), hw.obstacle_distance(), hw.heading(), cmd)
            hw.set_wheels(v - K * w, v + K * w)
            if hasattr(hw, "show"):
                hw.show(fsm.follower.points)
            log.writerow([f"{t:.3f}", fsm.state, f"{v:.4f}", f"{w:.4f}", f"{fsm.d:.3f}", fsm.line])
    return fsm


if __name__ == "__main__":
    from webots_hardware import WebotsHardware

    main(WebotsHardware(TIME_STEP))
