"""FSM of the ST5 robot (IDLE, FOLLOW, NODE, ALIGN, WAIT, SEARCH, TURN, CHECK) and its main loop. Writes fsm_log.csv."""
import csv
import math
import os
import time
from collections import deque

from line_follower import A_UP, K, LOST, OK, V0, LineFollower
from navigators import Explorer, Mission
from node_detector import ZONE, NodeProcess
from params import load

# values: params.ini [fsm]
MISSION = AUTO_START = TIME_STEP = None
D_STOP = D_CLEAR = D_SLOW = SLOW_FRACTION = T_STATIC = T_STALL = T_BLOCK = T_MIN = N_OK = None
SEARCH_RATE = TURN_RATE = TURN_EARLY = TURN_LATE = TURN_TOL_Y = SEARCH_TOL = D_TURN_LINE = T_TURN_LAG = None
A_BRAKE = WHEEL_LAG = K_HOLD = D_STRAIGHT = NODES = None
V_CREEP = NODE_TOL = T_SETTLE = T_SCAN = D_SCAN = None
load("fsm", globals())
D_SEE = ZONE[0]                     # nodes are announced at the start of the detector zone
TURNS = {"LEFT": math.pi / 2, "RIGHT": -math.pi / 2, "U_TURN": math.pi}

IDLE, FOLLOW, NODE, ALIGN, WAIT, SEARCH, TURN = "IDLE", "FOLLOW", "NODE", "ALIGN", "WAIT", "SEARCH", "TURN"
CHECK = "CHECK"                     # turn done at a node: still, the sonar reads the exit ahead
SPINNING = (TURN, SEARCH, CHECK)    # in place: s does not move, so anything seen would confirm as a node
HEADING = {"STRAIGHT": 0.0, **TURNS}   # direction of each order from the arrival heading
SONAR_FAR = 1.5                     # m: farthest sonar echo trusted to place an obstacle on the map (wall: 1.75)
STALL_V, STALL_W = 0.03, 0.3        # commands above these (m/s, rad/s) must move the wheels...
STALL_DS, STALL_DH = 0.001, 0.01    # ...by at least this much per cycle (m, rad); else T_STALL ends the run


class FSM:
    def __init__(self, follower, nav=Explorer):
        self.follower, self.make_nav, self.nav = follower, nav, nav()
        self.state, self.reason, self.history = IDLE, "start", []
        self.t = self.t_enter = self.heading = self.heading0 = self.s = 0.0
        self.v = self.w = 0.0
        self.d, self.dists = float("inf"), deque(maxlen=3)
        self.line, self.side = LOST, 1.0
        self.psi_exit, self.s_turn = None, -math.inf   # heading the last turn aimed at, and where it ended (s)
        self.psi_search, self.search_dir = None, 1.0   # SEARCH: heading the line must be near (None: any), spin sign
        self.n_ok, self.saved, self.turn_goal, self.y_prev = 0, FOLLOW, math.pi, None
        self.s_node = self.psi_ref = 0.0
        self.order, self.why, self.node_event, self.exits = "STOP", "", None, None
        self.check_exit = False             # the turn ends at a node exit (CHECK: the sonar reads it)
        self.ds, self.t_still, self.t_stop, self.fix, self.fixed, self.t_fix_end = 0.0, 0.0, 0.0, 0, False, 0.0   # ALIGN
        self.t_moved = 0.0                  # last time the wheels did what was commanded (stall check)

    def step(self, t, image, dist, heading, cmd, s=0.0, node=None):
        self.ds = s - self.s                # distance moved since the last cycle (ALIGN: are the wheels still?)
        moved = abs(self.ds) > STALL_DS or abs(heading - self.heading) > STALL_DH
        if moved or (abs(self.v) < STALL_V and abs(self.w) < STALL_W):
            self.t_moved = t                # the last command moved the robot, or asked for no motion
        self.t, self.heading, self.s, self.node_event = t, heading, s, node
        self.dists.append(dist)
        self.d = sorted(self.dists)[len(self.dists) // 2]
        if cmd == "STOP" and self.state != IDLE:
            self.go(IDLE, "STOP command")
        elif self.state != IDLE and t - self.t_moved > T_STALL:   # the logs showed it: FOLLOW for 12 s, standing still
            self.go(IDLE, f"STALL: motion commanded for {T_STALL:.1f} s but the wheels do not turn "
                          "(motor battery switched on late or flat? wheels blocked?)")
        elif self.state in (FOLLOW, NODE) and self.d < D_STOP:
            self.saved = self.state
            self.go(WAIT, f"obstacle at {self.d:.2f} m")
        self.v, self.w = getattr(self, self.state.lower())(image, cmd)
        return self.v, self.w

    def go(self, new, reason):
        print(f"[FSM] t={self.t:6.2f}s {self.state} -> {new} ({reason})")
        self.history.append((self.t, self.state, new, reason))
        self.state, self.reason = new, reason
        if new == SEARCH:
            self.aim_search()
        self.t_enter, self.heading0, self.n_ok = self.t, self.heading, 0

    def aim_search(self):
        """Line lost within D_TURN_LINE of a turn: the search spins back towards the heading the turn aimed at and
        takes only a line within SEARCH_TOL of it. Elsewhere on a grid mission: only a line within SEARCH_TOL of the
        heading it had when it lost the line. Else (the 8): any line, on the side last seen."""
        near = self.psi_exit is not None and self.s - self.s_turn < D_TURN_LINE
        grid = self.nav.prior() is not None
        self.psi_search = self.psi_exit if near else self.heading if grid else None
        off = 0.0 if self.psi_search is None else math.remainder(self.psi_search - self.heading, 2 * math.pi)
        self.search_dir = math.copysign(1.0, off) if abs(off) > SEARCH_TOL / 4 else self.side

    def look(self, image):
        out = self.follower.step(image, 0.0, 0.0, self.w)
        self.line = out.status
        return out

    def v_max(self):
        return V0 * min(1.0, max(SLOW_FRACTION, (self.d - D_STOP) / (D_SLOW - D_STOP)))

    def node_ahead(self, s_node, exits, reason, aim=0.0):
        self.s_node, self.psi_ref, self.exits = s_node, self.heading + aim, exits
        self.order, self.why = self.nav.on_node(exits, s_node)
        self.go(NODE, f"{reason}: {self.order} ({self.why})")

    def start_turn(self, theta, reason, check_exit=False):
        self.turn_goal, self.y_prev, self.check_exit = theta, None, check_exit and D_SCAN > 0
        self.go(TURN, reason)

    def idle(self, image, cmd):
        if cmd == "START":
            self.nav = self.make_nav()
            self.go(FOLLOW, "START command")
        elif isinstance(cmd, tuple) and cmd[0] == "MISSION":
            self.nav = Mission(*cmd[1:], t_block=T_BLOCK)
            self.node_ahead(self.s, None, f"MISSION {cmd[1:]}")
        return 0.0, 0.0

    def follow(self, image, cmd):
        out = self.follower.step(image, self.v_max(), self.v, self.w)
        self.line, self.side = out.status, out.side
        prior = self.nav.prior()            # mission: where the map puts the next node (s) and its exits
        if prior and self.d < SONAR_FAR:    # the sonar looks along the line (always straight ahead): an obstacle
            seen = self.nav.on_seen(self.s, self.d)                 # beyond the next node changes the plan now
            if seen:
                print(f"[FSM] t={self.t:6.2f}s {seen}")
        half = self.nav.g / 2 if prior else 0.0   # any node within half a spacing of it is that node: the detector
        if self.node_event:                 # says what is a node, the odometry only which one (wheel slip, a bad
            d, exits = self.node_event      # start or the camera error doubled by a U-turn stay well inside)
            if prior and abs(self.s + d - prior[0]) > half:
                print(f"[FSM] t={self.t:6.2f}s node at {d:.2f} m ignored: {self.s + d - prior[0]:+.2f} m from the "
                      f"next node of the map")
            else:
                aim = math.atan2(out.target[1], out.target[0]) if out.target else 0.0
                self.node_ahead(self.s + d, exits, f"node at {d:.2f} m, exits {''.join(sorted(exits))}", aim)
                return out.v, out.w
        elif prior and self.s > prior[0] + half - D_SEE:   # even a node half a spacing late would be seen by now
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
        """To the node centre, keeping the heading. Straight on: full speed. A turn: full speed, then braking at A_BRAKE
        so that it reaches the centre at V_CREEP, then ALIGN (stop) there."""
        if self.order == "STRAIGHT" and self.s >= self.s_node - D_STRAIGHT:
            self.go(FOLLOW, "bar passed")
        elif self.order != "STRAIGHT" and self.s >= self.s_node:
            if self.order in TURNS:
                self.fix, self.fixed, self.t_still, self.t_stop = 0, False, self.t, self.t
                self.go(ALIGN, f"at the node centre: stop before {self.order}")
            else:
                self.go(IDLE, f"{self.order} at the node: {self.why}")
        else:
            v = min(V0, self.v + A_UP * TIME_STEP / 1000)   # straight on: no reason to slow down
            if self.order != "STRAIGHT":    # a turn: the fastest speed that can still stop on the centre, the wheels
                left = self.s_node - self.s - self.v * WHEEL_LAG   # going on at this speed for WHEEL_LAG first
                v = min(v, math.sqrt(V_CREEP ** 2 + 2 * A_BRAKE * max(0.0, left)))
            return min(v, self.v_max()), K_HOLD * (self.psi_ref - self.heading)
        return 0.0, 0.0

    def align(self, image, cmd):
        """Stopped at the node centre before a turn: turn once the wheels are still for T_SETTLE, so it spins in
        place on the node. Stopped farther than NODE_TOL from the centre: one slow move forward or back first."""
        off = self.s - self.s_node                 # > 0: past the centre
        hold = K_HOLD * (self.psi_ref - self.heading)
        if self.fix:                               # slow move back to the centre, then stop again
            if off * self.fix >= 0.0 or self.t >= self.t_fix_end:
                self.fix, self.t_still, self.t_stop = 0, self.t, self.t
                return 0.0, 0.0
            return self.fix * V_CREEP, hold
        if abs(self.ds) > V_CREEP * TIME_STEP / 2000:    # still rolling (slower than V_CREEP / 2 = still)
            self.t_still = self.t
        if self.t - self.t_still < T_SETTLE and self.t - self.t_stop < 3 * T_SETTLE:   # (noisy odometry: at most 3x)
            return 0.0, 0.0
        if abs(off) > NODE_TOL and not self.fixed:
            self.fix, self.fixed = (1 if off < 0 else -1), True
            self.t_fix_end = self.t + 2 * abs(off) / V_CREEP + 1.0
            print(f"[FSM] t={self.t:6.2f}s stopped {100 * off:+.1f} cm from the node centre: "
                  f"moving {'forward' if off < 0 else 'back'}")
            return 0.0, 0.0
        self.start_turn(TURNS[self.order], f"stopped {100 * off:+.1f} cm from the node centre: {self.order}", True)
        return 0.0, 0.0

    def check(self, image, cmd):
        """Turn done at a node: still for T_SCAN while the sonar, now facing the exit, gets fresh readings.
        Clear: FOLLOW. Closer than D_SCAN: the navigator drops that exit and the robot turns to the one it chooses."""
        if self.t - self.t_enter < T_SCAN:
            return 0.0, 0.0
        if self.d >= D_SCAN:
            left = self.angle_left(self.follower.find_target(image))   # still: where the turn really ended
            end = "" if left is None else f"; the turn ended {abs(math.degrees(left)):.0f} deg " \
                                          f"{'short of' if left > 0 else 'past'} the line"
            self.go(FOLLOW, f"exit clear ({self.d:.2f} m){end}")
            self.node_from_map()
            return 0.0, 0.0
        old = self.order
        order, why = self.nav.on_exit_blocked(self.exits)
        reason = f"{old} exit blocked at {self.d:.2f} m: {order} ({why})"
        if order == "GO_ON":                       # navigator without a map: FOLLOW, the obstacle stop does the rest
            self.go(FOLLOW, reason)
        elif order in HEADING:
            self.order = order
            # the shorter way: RIGHT then U_TURN is 90 deg more to the right, not 270 to the left
            self.start_turn(math.remainder(HEADING[order] - HEADING[old], 2 * math.pi), reason, True)
        else:
            self.go(IDLE, reason)
        return 0.0, 0.0

    def wait(self, image, cmd):
        if self.d > D_CLEAR:                       # hysteresis over D_STOP
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
        aimed = self.psi_search is None or \
            abs(math.remainder(self.heading - self.psi_search, 2 * math.pi)) <= SEARCH_TOL
        if self.look(image).status == OK and aimed:
            self.n_ok += 1
            if self.n_ok >= N_OK and self.t - self.t_enter >= T_MIN:
                self.go(FOLLOW, "line found")
                self.node_from_map()
            return 0.0, 0.0
        self.n_ok = 0
        if abs(self.heading - self.heading0) >= 2 * math.pi:
            self.go(IDLE, "line lost (search exhausted)")
            return 0.0, 0.0
        return 0.0, self.search_dir * SEARCH_RATE

    def turn(self, image, cmd):
        """Spin until the line is centred, from |goal| - TURN_EARLY to |goal| + TURN_LATE (the wheels slip).
        Stopped early: the robot still turns TURN_RATE * T_TURN_LAG after the stop (the image is late, the wheels
        stop late), so the spin ends when the line is that angle away. T_TURN_LAG = 0: centred = N_OK frames within
        TURN_TOL_Y, or y changing sign near the axis (the line crossed the centre between two frames)."""
        turned, goal = abs(self.heading - self.heading0), abs(self.turn_goal)
        if turned >= goal - TURN_EARLY:
            target = self.follower.find_target(image)
            self.line = LOST if target is None else OK
            y = None if target is None else target[1]
            near = y is not None and abs(y) <= 2 * TURN_TOL_Y
            crossed = near and self.y_prev is not None and y * self.y_prev < 0
            self.y_prev = y if near else None
            self.n_ok = self.n_ok + 1 if y is not None and abs(y) <= TURN_TOL_Y else 0
            left = self.angle_left(target)         # still to turn to the line (rad, < 0: past it)
            early = left is not None and abs(left) <= TURN_RATE * T_TURN_LAG
            if crossed or self.n_ok >= N_OK or early:
                how = (f"stopped {math.degrees(left):.0f} deg before the line (lag)" if early else
                       "line crossed the centre" if crossed else "line ahead")
                done = f"turn done after {math.degrees(turned):.0f} deg, {how}"
                self.end_turn(CHECK if self.check_exit else FOLLOW, done)
                if not self.check_exit:
                    self.node_from_map()
                return 0.0, 0.0
            if self.n_ok:                          # centred: hold still while confirming
                return 0.0, 0.0
        if turned >= goal + TURN_LATE:
            order, _ = self.nav.on_fault("turn failed")
            self.end_turn(IDLE if order == "STOP" else SEARCH, f"turned {math.degrees(turned):.0f} deg without the line")
            return 0.0, 0.0
        return 0.0, math.copysign(TURN_RATE, self.turn_goal)

    def angle_left(self, target):
        """Angle still to turn to the line seen at target (L ahead, y left), in the direction of the turn; None if
        no line. > 0: not there yet, < 0: past it."""
        return None if target is None else math.copysign(1.0, self.turn_goal) * math.atan2(target[1], target[0])

    def node_from_map(self):
        """Back to FOLLOW after a turn or a search: a next node closer than the camera zone is never seen, take it from the map."""
        prior = self.nav.prior()
        if prior and 0.0 <= prior[0] - self.s < D_SEE:
            self.node_ahead(prior[0], None, f"node {prior[0] - self.s:.2f} m ahead")

    def end_turn(self, new, reason):
        self.psi_exit, self.s_turn = self.heading0 + self.turn_goal, self.s   # heading0: where the turn began
        self.dists.clear()
        self.follower.reset()
        self.go(new, reason)


STAGES = ("capture", "nodes", "fsm", "io", "view")   # parts of one loop cycle, timed by LoopTimer
WATCHDOG_MS = 300   # robo_fsm.ino TIMEOUT_MS: no wheel command for this long and the Arduino stops the motors


class LoopTimer:
    """Where the loop time goes: ms of each stage of a cycle (fsm_log.csv) and a summary at exit.
    period = start to start of two cycles (target TIME_STEP); busy = work in the cycle, without the wait for the tick."""

    def __init__(self):
        self.t = self.t_start = None
        self.ms, self.cycles = {}, []      # cycles: (period, busy, ms of each stage)

    def start(self):
        now = time.perf_counter()
        self.period = 1000 * (now - self.t_start) if self.t_start is not None else None
        self.t_start = self.t = now
        self.ms = dict.fromkeys(STAGES, 0.0)

    def lap(self, stage):
        now = time.perf_counter()
        self.ms[stage] += 1000 * (now - self.t)
        self.t = now

    def row(self):
        """[period, busy, stages...] in ms for the log; the cycle is kept for the summary."""
        busy = sum(self.ms.values())
        if self.period is not None:
            self.cycles.append((self.period, busy, [self.ms[k] for k in STAGES]))
        return ["" if self.period is None else f"{self.period:.1f}", f"{busy:.1f}"] + [f"{self.ms[k]:.1f}" for k in STAGES]

    def report(self):
        if not self.cycles:
            return
        period, busy, stages = zip(*self.cycles)
        late = sum(p > 1.2 * TIME_STEP for p in period)
        print(f"[LOOP] {len(period)} cycles: period mean {sum(period) / len(period):.0f} ms, max {max(period):.0f} ms "
              f"(target {TIME_STEP} ms, {100 * late / len(period):.0f}% over +20%); "
              f"busy mean {sum(busy) / len(busy):.0f} ms, max {max(busy):.0f} ms")
        for k, ms in zip(STAGES, zip(*stages)):
            print(f"[LOOP]   {k:8s} mean {sum(ms) / len(ms):5.1f} ms, max {max(ms):5.1f} ms")
        stalls = sum(p > WATCHDOG_MS for p in period)
        if stalls:
            print(f"[LOOP] WARNING: {stalls} cycle(s) over {WATCHDOG_MS} ms: the Arduino stopped the motors each time "
                  f"(jerks). The serial link may need a core of its own")


def check_sonar(fsm, hw):
    """Sonar without signal: stop with that reason, instead of taking it for an obstacle at 0 m (U-turns forever)."""
    if fsm.state != IDLE and hasattr(hw, "sonar_ok") and not hw.sonar_ok():
        fsm.go(IDLE, "sonar: no signal (cable unplugged?)")


def look_for_node(detector, fsm, image, s):
    """detector.step(), except while the robot spins in place (the node being confirmed is dropped then);
    a node is announced only in FOLLOW."""
    if fsm.state in SPINNING:
        detector.forget()
        return None
    return detector.step(image, s, fsm.state == FOLLOW)


def main(hw, nav=Explorer):
    width, height, fov = hw.camera_info()
    fsm = FSM(LineFollower(width, height, fov, TIME_STEP / 1000.0, hw.wheel_v_max()), nav)
    if hasattr(hw, "attach"):           # e.g. cli.py: watches the FSM to know when a delivery ends
        hw.attach(fsm)
    detector = None                     # MISSION = None (test of the 8): line and obstacles only
    if MISSION is not None:             # grid mission: the intersections (nodes) are needed
        detector = NodeProcess(width, height, fov, fsm.follower.grey)    # another core; one grey image per frame
        if NODES != "bars":             # corner detection v4 (corner_detector.py): "+" crossings
            from corner_detector import CornerDetector, NodeFusion
            grey = fsm.follower.grey if fsm.follower.px_step == 1 else None
            corners = CornerDetector(width, height, fov, grey=grey)
            detector = corners if NODES == "corners" else NodeFusion(detector, corners)
    else:
        print("[FSM] no MISSION (test of the 8): follows the line and stops for obstacles; "
              "nodes and corners are not searched")
    path = os.environ.get("FSM_LOG") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "fsm_log.csv")
    timer = LoopTimer()
    try:
        with open(path, "w", newline="", buffering=1) as f:
            log = csv.writer(f)
            log.writerow(["t", "state", "v", "w", "dist", "line", "s", "node", "period_ms", "busy_ms"]
                         + [f"{k}_ms" for k in STAGES])
            while hw.step():
                timer.start()
                t, cmd = hw.time(), hw.command()
                if AUTO_START is not None and not fsm.history and t >= AUTO_START:
                    cmd = cmd or MISSION or "START"
                image, s = hw.camera_image(), hw.distance()
                timer.lap("capture")
                node = None             # nodes are only used (announced) in FOLLOW, and only with a MISSION
                if detector is not None:
                    node = look_for_node(detector, fsm, image, s)
                if node:
                    print(f"[NODE] t={t:6.2f}s s={s:.3f} d={node[0]:.3f} exits={sorted(node[1])}")
                timer.lap("nodes")
                fsm.follower.points = []
                check_sonar(fsm, hw)
                v, w = fsm.step(t, image, hw.obstacle_distance(), hw.heading(), cmd, s, node)
                timer.lap("fsm")
                hw.set_wheels(v - K * w, v + K * w)   # the sonar servo stays straight ahead
                timer.lap("io")
                if hasattr(hw, "show"):
                    hw.show(fsm.follower.points, fsm.follower.cached_mask(image))
                timer.lap("view")
                log.writerow([f"{t:.3f}", fsm.state, f"{v:.4f}", f"{w:.4f}", f"{fsm.d:.3f}", fsm.line, f"{s:.3f}",
                              f"{node[0]:.3f} {''.join(sorted(node[1]))}" if node else ""] + timer.row())
    finally:                            # also on Ctrl+C: the summary is the point of a test drive
        timer.report()
    return fsm


if __name__ == "__main__":
    from robot_hardware import RobotHardware

    try:
        main(RobotHardware(TIME_STEP))
    except KeyboardInterrupt:           # Ctrl+C: the normal way to stop (the motors stop on exit)
        print("\nstopped (Ctrl+C)")
