"""fsm.FSM SEARCH right after a turn: it looks for the line the turn aimed at, not the perpendicular one."""
import math
import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Robo_FSM", "pi"))

import fsm
from fsm import FOLLOW, FSM, SEARCH


class FakeFollower:
    """The line is seen (OK) only when `visible` is True."""

    def __init__(self):
        self.visible = False

    def step(self, image, v, v_now, w):
        return SimpleNamespace(status=fsm.OK if self.visible else fsm.LOST, side=1.0)

    def reset(self):
        pass


def after_turn(turned_deg):
    """An FSM that just turned 90 deg left (it actually turned `turned_deg`), then lost the line."""
    f = FSM(FakeFollower())
    f.heading0, f.turn_goal, f.heading = 0.0, math.pi / 2, math.radians(turned_deg)
    f.end_turn(FOLLOW, "turn done")
    f.go(SEARCH, "line lost")
    return f


def see(f, heading_deg, frames):
    f.follower.visible, f.heading = True, math.radians(heading_deg)
    for _ in range(frames):
        f.t += 0.1
        f.search(None, None)


class TestSearchAfterTurn(unittest.TestCase):
    def test_overshoot_spins_back_towards_the_turn_heading(self):
        f = after_turn(107)
        self.assertEqual(f.search_dir, -1.0)
        self.assertAlmostEqual(f.psi_search, math.pi / 2)

    def test_perpendicular_line_is_not_taken(self):
        f = after_turn(107)
        see(f, 180, fsm.N_OK + 5)
        self.assertEqual(f.state, SEARCH)

    def test_line_near_the_turn_heading_is_taken(self):
        f = after_turn(107)
        see(f, 95, fsm.N_OK + 5)
        self.assertEqual(f.state, FOLLOW)

    def test_far_from_a_turn_any_line_on_the_last_side(self):
        f = FSM(FakeFollower())
        f.side = -1.0
        f.go(SEARCH, "line lost")
        self.assertIsNone(f.psi_search)
        self.assertEqual(f.search_dir, -1.0)
        see(f, 180, fsm.N_OK + 5)
        self.assertEqual(f.state, FOLLOW)

    def test_far_after_a_turn_any_line(self):
        f = FSM(FakeFollower())
        f.heading0, f.turn_goal = 0.0, math.pi / 2
        f.end_turn(FOLLOW, "turn done")
        f.s = fsm.D_TURN_LINE + 0.1        # it drove on: a later line loss is not about the turn
        f.go(SEARCH, "line lost")
        self.assertIsNone(f.psi_search)


if __name__ == "__main__":
    unittest.main()


class LineAt(FakeFollower):
    """find_target: the line `deg` degrees to the left of the robot axis, 0.30 m ahead (None: no line)."""

    def __init__(self, deg=None):
        super().__init__()
        self.deg = deg

    def find_target(self, image):
        return None if self.deg is None else (0.30, 0.30 * math.tan(math.radians(self.deg)))


def turning_left(turned_deg, line_deg):
    f = FSM(LineAt(line_deg))
    f.start_turn(math.pi / 2, "LEFT")
    f.heading = math.radians(turned_deg)
    f.t += 0.1
    return f, f.turn(None, None)


class TestTurnLag(unittest.TestCase):
    """The spin stops TURN_RATE * T_TURN_LAG before the line: the robot goes on turning that much after the stop."""

    def lead_deg(self):
        return math.degrees(fsm.TURN_RATE * fsm.T_TURN_LAG)

    def test_stops_before_the_line_by_the_lag(self):
        f, (_, w) = turning_left(80, self.lead_deg() - 3)
        self.assertEqual((f.state, w), (FOLLOW, 0.0))
        self.assertIn("before the line (lag)", f.reason)

    def test_line_still_far_keeps_spinning(self):
        f, (_, w) = turning_left(60, self.lead_deg() + 10)
        self.assertEqual(f.state, fsm.TURN)
        self.assertGreater(w, 0.0)

    def test_angle_left_sign(self):
        f = FSM(LineAt())
        f.turn_goal = -math.pi / 2              # a right turn: a line on the right is still ahead
        self.assertGreater(f.angle_left((0.3, -0.05)), 0.0)
        self.assertLess(f.angle_left((0.3, 0.05)), 0.0)
        self.assertIsNone(f.angle_left(None))


class TestXCamFromStops(unittest.TestCase):
    def test_stopped_past_the_node_moves_the_estimate_nearer(self):
        sys.modules.setdefault("libcamera", SimpleNamespace(Transform=None))
        import diag
        # odometry: on the centre (+0.3 cm); tape: 3 cm past it -> the node was placed 2.7 cm too far
        self.assertAlmostEqual(diag.x_cam_for([0.003 - 0.03]), diag.X_CAM - 0.027)


class GridNav:
    """A mission's navigator, as far as SEARCH and WAIT need it: a next node on the map."""

    def prior(self):
        return (5.0, {"L", "R", "S"})

    def on_clear(self):
        pass

    def on_blocked(self, s, d):
        return "U_TURN", "edge blocked"


class TestGridSearchAndObstacle(unittest.TestCase):
    """On the grid the search keeps the heading of the line; a box read near D_STOP still blocks."""

    def grid_fsm(self, heading_deg):
        f = FSM(FakeFollower(), nav=GridNav)
        f.heading = math.radians(heading_deg)
        f.s = 3.0                                 # far from any turn
        return f

    def test_grid_search_takes_only_the_line_it_was_on(self):
        f = self.grid_fsm(90)
        f.go(SEARCH, "line lost")
        self.assertAlmostEqual(f.psi_search, math.pi / 2)
        see(f, 135 + 5, fsm.N_OK + 5)             # a diagonal: refused
        self.assertEqual(f.state, SEARCH)
        see(f, 100, fsm.N_OK + 5)                 # the line it was on
        self.assertEqual(f.state, FOLLOW)

    def test_box_read_just_above_the_stop_distance_still_blocks(self):
        f = self.grid_fsm(90)
        f.saved = FOLLOW
        f.go(fsm.WAIT, "obstacle")
        f.d = fsm.D_STOP + 0.02                   # askew: the echo slips off the box a little
        f.wait(None, None)
        self.assertEqual(f.state, fsm.WAIT)
        f.t += fsm.T_STATIC
        f.wait(None, None)
        self.assertEqual(f.state, fsm.TURN)       # static: U-turn, not over the box

    def test_obstacle_gone_resumes(self):
        f = self.grid_fsm(90)
        f.saved = FOLLOW
        f.go(fsm.WAIT, "obstacle")
        f.d = float("inf")
        f.wait(None, None)
        self.assertEqual(f.state, FOLLOW)


class TestSonarLost(unittest.TestCase):
    """No sonar signal (cable unplugged): stop with that reason, not U-turns at an 'obstacle at 0 m'."""

    def test_no_signal_stops(self):
        f = FSM(FakeFollower())
        f.go(FOLLOW, "START command")
        fsm.check_sonar(f, SimpleNamespace(sonar_ok=lambda: False))
        self.assertEqual(f.state, fsm.IDLE)
        self.assertIn("sonar: no signal", f.reason)

    def test_signal_or_no_sonar_check_goes_on(self):
        f = FSM(FakeFollower())
        f.go(FOLLOW, "START command")
        fsm.check_sonar(f, SimpleNamespace(sonar_ok=lambda: True))
        fsm.check_sonar(f, SimpleNamespace())          # Webots: no sonar_ok
        self.assertEqual(f.state, FOLLOW)

    def test_hardware_sees_the_no_signal_lines(self):
        sys.modules.setdefault("libcamera", SimpleNamespace(Transform=None))
        import robot_hardware as rh
        hw = rh.RobotHardware.__new__(rh.RobotHardware)
        hw.buf, hw.n_d, hw.counts, hw.counts0, hw.pwm, hw.alive = b"", 0, None, None, None, False
        hw.t_echo = rh.time.monotonic() - 2 * rh.SONAR_LOST_S
        hw.ser = SimpleNamespace(in_waiting=0)
        hw.buf = b"D,-2,1,2,3,4\n"
        hw._read_serial()
        self.assertFalse(hw.sonar_ok())
        hw.buf = b"D,35,1,2,3,4\n"
        hw._read_serial()
        self.assertTrue(hw.sonar_ok())
