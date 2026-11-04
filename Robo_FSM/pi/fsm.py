"""FSM of the ST5 robot (IDLE, FOLLOW, NODE, WAIT, SEARCH, TURN) and its main loop. Writes fsm_log.csv."""
import csv
import math
import os
from collections import deque

from line_follower import LOST, OK, V0, K, LineFollower
from navigators import Explorer, Mission
from node_detector import ZONE, NodeDetector
from params import load

# values: params.ini [fsm]
MISSION = NAV_ON_ARDUINO = AUTO_START = TIME_STEP = None
D_STOP = D_SLOW = SLOW_FRACTION = T_STATIC = T_MIN = N_OK = None
SEARCH_RATE = TURN_RATE = TURN_EARLY = TURN_LATE = TURN_TOL_Y = None
V_NODE = K_HOLD = D_STRAIGHT = PRIOR_TOL = NODES = None
load("fsm", globals())
D_SEE = ZONE[0]                     # nodes are announced at the start of the detector zone
TURNS = {"LEFT": math.pi / 2, "RIGHT": -math.pi / 2, "U_TURN": math.pi}

IDLE, FOLLOW, NODE, WAIT, SEARCH, TURN = "IDLE", "FOLLOW", "NODE", "WAIT", "SEARCH", "TURN"


class FSM:
    def __init__(self, follower, nav=Explorer):
        self.follower, self.make_nav, self.nav = follower, nav, nav()
        self.state, self.reason, self.history = IDLE, "start", []
        self.t = self.t_enter = self.heading = self.heading0 = self.s = 0.0
        self.v = self.w = 0.0
        self.d, self.dists = float("inf"), deque(maxlen=3)
        self.line, self.side = LOST, 1.0
        self.n_ok, self.saved, self.turn_goal, self.y_prev = 0, FOLLOW, math.pi, None
        self.s_node = self.psi_ref = 0.0
        self.order, self.why, self.node_event = "STOP", "", None

    def step(self, t, image, dist, heading, cmd, s=0.0, node=None):
        self.t, self.heading, self.s, self.node_event = t, heading, s, node
        self.dists.append(dist)
        self.d = sorted(self.dists)[len(self.dists) // 2]
        if cmd == "STOP" and self.state != IDLE:
            self.go(IDLE, "STOP command")
        elif self.state in (FOLLOW, NODE) and self.d < D_STOP:
            self.saved = self.state
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

    def v_max(self):
        return V0 * min(1.0, max(SLOW_FRACTION, (self.d - D_STOP) / (D_SLOW - D_STOP)))

    def node_ahead(self, s_node, exits, reason, aim=0.0):
        self.s_node, self.psi_ref = s_node, self.heading + aim
        self.order, self.why = self.nav.on_node(exits, s_node)
        self.go(NODE, f"{reason}: {self.order} ({self.why})")

    def start_turn(self, theta, reason):
        self.turn_goal, self.y_prev = theta, None
        self.go(TURN, reason)

    def idle(self, image, cmd):
        if cmd == "START":
            self.nav = self.make_nav()
            self.go(FOLLOW, "START command")
        elif isinstance(cmd, tuple) and cmd[0] == "MISSION":
            self.nav = Mission(*cmd[1:])
            self.node_ahead(self.s, None, f"MISSION {cmd[1:]}")
        return 0.0, 0.0

    def follow(self, image, cmd):
        out = self.follower.step(image, self.v_max(), self.v, self.w)
        self.line, self.side = out.status, out.side
        prior = self.nav.prior()
        if self.node_event:
            d, exits = self.node_event
            if prior and abs(self.s + d - prior[0]) > PRIOR_TOL:
                print(f"[FSM] t={self.t:6.2f}s false node at {d:.2f} m ignored")
            else:
                aim = math.atan2(out.target[1], out.target[0]) if out.target else 0.0
                self.node_ahead(self.s + d, exits, f"node at {d:.2f} m, exits {''.join(sorted(exits))}", aim)
                return out.v, out.w
        elif prior and self.s > prior[0] - D_SEE + PRIOR_TOL:   # should have been seen by now
            order, why = self.nav.on_fault("node missed")
            print(f"[FSM] t={self.t:6.2f}s {why}")
            if order == "STOP":
                self.go(IDLE, why)
                return 0.0, 0.0
        if out.status == LOST and self.t - self.t_enter >= T_MIN:
            self.go(SEARCH, "line lost")
            return 0.0, 0.0
        return out.v, out.w

    def node(self, image, cmd):
        if self.order == "STRAIGHT" and self.s >= self.s_node - D_STRAIGHT:
            self.go(FOLLOW, "bar passed")
        elif self.order != "STRAIGHT" and self.s >= self.s_node:
            if self.order in TURNS:
                self.start_turn(TURNS[self.order], f"at the node centre: {self.order}")
            else:
                self.go(IDLE, f"{self.order} at the node: {self.why}")
        else:
            return min(V_NODE, self.v_max()), K_HOLD * (self.psi_ref - self.heading)
        return 0.0, 0.0

    def wait(self, image, cmd):
        if self.d > D_STOP:
            self.nav.on_clear()
            self.go(self.saved, "obstacle left: not static")
        elif self.t - self.t_enter >= T_STATIC:
            order, why = self.nav.on_blocked(self.s, self.d)
            if order == "U_TURN":
                self.start_turn(math.pi, why)
            elif order == "WAIT":
                self.t_enter = self.t
            else:
                self.go(IDLE, why)
        return 0.0, 0.0

    def search(self, image, cmd):
        if self.look(image).status == OK:
            self.n_ok += 1
            if self.n_ok >= N_OK and self.t - self.t_enter >= T_MIN:
                self.go(FOLLOW, "line found")
                self.node_from_map()
            return 0.0, 0.0
        self.n_ok = 0
        if abs(self.heading - self.heading0) >= 2 * math.pi:
            self.go(IDLE, "line lost (search exhausted)")
            return 0.0, 0.0
        return 0.0, self.side * SEARCH_RATE

    def turn(self, image, cmd):
        """Spin until the line is centred, from |goal| - TURN_EARLY to |goal| + TURN_LATE (the wheels slip).
        Centred: N_OK frames within TURN_TOL_Y, or y changing sign near the axis (the line crossed the centre
        between two frames: with camera delay it can pass the TURN_TOL_Y window in fewer than N_OK frames)."""
        turned, goal = abs(self.heading - self.heading0), abs(self.turn_goal)
        if turned >= goal - TURN_EARLY:
            target = self.follower.find_target(image)
            self.line = LOST if target is None else OK
            y = None if target is None else target[1]
            near = y is not None and abs(y) <= 2 * TURN_TOL_Y
            crossed = near and self.y_prev is not None and y * self.y_prev < 0
            self.y_prev = y if near else None
            self.n_ok = self.n_ok + 1 if y is not None and abs(y) <= TURN_TOL_Y else 0
            if crossed or self.n_ok >= N_OK:
                how = "line crossed the centre" if crossed else "line ahead"
                self.end_turn(FOLLOW, f"turn done after {math.degrees(turned):.0f} deg, {how}")
                self.node_from_map()
                return 0.0, 0.0
            if self.n_ok:                          # centred: hold still while confirming
                return 0.0, 0.0
        if turned >= goal + TURN_LATE:
            order, _ = self.nav.on_fault("turn failed")
            self.end_turn(IDLE if order == "STOP" else SEARCH, f"turned {math.degrees(turned):.0f} deg without the line")
            return 0.0, 0.0
        return 0.0, math.copysign(TURN_RATE, self.turn_goal)

    def node_from_map(self):
        """Back to FOLLOW after a turn or a search: a next node closer than the camera zone is never seen, take it from the map."""
        prior = self.nav.prior()
        if prior and 0.0 <= prior[0] - self.s < D_SEE:
            self.node_ahead(prior[0], None, f"node {prior[0] - self.s:.2f} m ahead")

    def end_turn(self, new, reason):
        self.dists.clear()
        self.follower.reset()
        self.go(new, reason)


def main(hw, nav=Explorer):
    global Mission
    if NAV_ON_ARDUINO:                  # the grid mission runs on the Arduino; FSM.idle() creates it as before
        from arduino_navigator import ArduinoMission
        Mission = lambda *args: ArduinoMission(hw, *args)  # noqa: E731
    width, height, fov = hw.camera_info()
    fsm = FSM(LineFollower(width, height, fov, TIME_STEP / 1000.0, hw.wheel_v_max()), nav)
    detector = NodeDetector(width, height, fov, fsm.follower.mask)
    if NODES != "bars":                 # corner detection v4 (corner_detector.py): "+" crossings
        from corner_detector import CornerDetector, NodeFusion
        corners = CornerDetector(width, height, fov)
        detector = corners if NODES == "corners" else NodeFusion(detector, corners)
    path =os.environ.get("FSM_LOG") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "fsm_log.csv")
    with open(path, "w", newline="", buffering=1) as f:
        log = csv.writer(f)
        log.writerow(["t", "state", "v", "w", "dist", "line", "s", "node"])
        while hw.step():
            t, cmd = hw.time(), hw.command()
            if AUTO_START is not None and not fsm.history and t >= AUTO_START:
                cmd = cmd or MISSION or "START"
            image, s = hw.camera_image(), hw.distance()
            node = detector.step(image, s)
            if node:
                print(f"[NODE] t={t:6.2f}s s={s:.3f} d={node[0]:.3f} exits={sorted(node[1])}")
            fsm.follower.points = []
            v, w = fsm.step(t, image, hw.obstacle_distance(), hw.heading(), cmd, s, node)
            hw.set_wheels(v - K * w, v + K * w)
            if hasattr(hw, "show"):
                hw.show(fsm.follower.points)
            log.writerow([f"{t:.3f}", fsm.state, f"{v:.4f}", f"{w:.4f}", f"{fsm.d:.3f}", fsm.line, f"{s:.3f}",
                          f"{node[0]:.3f} {''.join(sorted(node[1]))}" if node else ""])
    return fsm


if __name__ == "__main__":
    from robot_hardware import RobotHardware

    main(RobotHardware(TIME_STEP))
