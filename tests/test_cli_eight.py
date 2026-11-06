"""cli.py --eight without the robot: the drawing, the position on the 8, the lap count (a fake FSM)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Robo_FSM", "pi"))

from rich.console import Console  # noqa: E402

from cli import run_params, ini_value  # noqa: E402
from cli_eight import (CROSSING, LOOP_A, LOOP_B, SETTINGS, W, EightCli, EightPosition,  # noqa: E402
                       FakeEightFsm, eight_panel)

PARAMS_INI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Robo_FSM", "pi", "params.ini")


def quiet(**kw):
    return EightCli(ask=None, say=lambda *a: None, live=False, **kw)


def drive(cli, fake, until, dt=0.064, limit=5000):
    """The robot loop, sped up: one cycle per call, fake time."""
    cli.attach(fake)
    t = 0.0
    for _ in range(limit):
        if until():
            return t
        fake.step(cli.command(), t, dt)
        t += dt
    raise AssertionError("never got there")


class TestDrawing(unittest.TestCase):
    def test_two_loops_meet_at_the_crossing_in_the_middle(self):
        self.assertEqual(CROSSING, ((W - 1) // 2, CROSSING[1]))
        self.assertTrue(all(c[0] > CROSSING[0] for c in LOOP_A[3:-3]))    # one loop on each side
        self.assertTrue(all(c[0] < CROSSING[0] for c in LOOP_B[3:-3]))
        for loop in (LOOP_A, LOOP_B):                                     # each loop is a continuous line
            for a, b in zip(loop, loop[1:]):
                self.assertLessEqual(max(abs(a[0] - b[0]), abs(a[1] - b[1])), 1)

    def test_panel_shows_the_car_and_the_lap(self):
        console = Console(record=True, width=80)
        console.print(eight_panel((0, 0.5, True, 3, 6, "FOLLOW", 0.3, 9.9, 0, True, 2.4, 12.0, 4.2)))
        text = console.export_text()
        self.assertIn("●", text)
        self.assertIn("lap 2 of 3", text)
        self.assertIn("following the line", text)
        self.assertIn("4.2 m", text)                                      # distance driven this run


class TestPosition(unittest.TestCase):
    def test_unknown_until_the_first_crossing(self):
        pos = EightPosition()
        self.assertIsNone(pos.fraction(1.0))
        pos.crossing(1.0)
        self.assertEqual(pos.loop, 0)
        self.assertLess(pos.fraction(1.1), pos.fraction(2.0))             # moves along the loop

    def test_second_crossing_measures_the_loop_and_switches_loops(self):
        pos = EightPosition()
        pos.crossing(1.0)
        pos.crossing(3.5)
        self.assertEqual((pos.loop, pos.loop_m, pos.measured), (1, 2.5, True))

    def test_u_turn_reverses_and_does_not_measure(self):
        pos = EightPosition(loop_m=2.0)
        pos.crossing(0.0)
        pos.u_turn(1.0)                                                   # halfway: turns back
        self.assertGreater(pos.fraction(1.0), pos.fraction(1.5))          # now going back to the crossing
        pos.crossing(2.0)
        self.assertFalse(pos.measured)                                    # not a whole loop
        self.assertEqual(pos.loop, 1)
        self.assertGreater(pos.fraction(2.2), pos.fraction(2.6))          # still reversed, in the other loop


class TestLaps(unittest.TestCase):
    def test_one_lap_then_stop(self):
        cli, fake = quiet(), FakeEightFsm(v=0.5)
        cli.handle("1")
        drive(cli, fake, lambda: cli.runs)
        self.assertEqual(cli.runs[0][:3], (1, 1, 0))                      # asked 1, did 1, no U-turn
        self.assertEqual(cli.runs[0][4], "done")
        self.assertEqual(fake.state, "IDLE")
        self.assertEqual(cli.pos.passes, 2)                               # 2 crossings = 1 lap

    def test_next_run_goes_on_from_there(self):
        cli, fake = quiet(), FakeEightFsm(v=0.5)
        cli.handle("1")
        drive(cli, fake, lambda: cli.runs)
        cli.handle("2")
        drive(cli, fake, lambda: len(cli.runs) == 2)
        self.assertEqual(cli.runs[1][:2], (2, 2))
        self.assertEqual(cli.pos.passes, 6)

    def test_obstacle_gives_a_u_turn(self):
        cli, fake = quiet(on_obstacle=None), FakeEightFsm(v=0.5, t_static=0.5)
        cli.handle("1")
        drive(cli, fake, lambda: fake.s > 0.3)
        fake.obstacle()
        drive(cli, fake, lambda: cli.runs)
        self.assertEqual(cli.runs[0][2], 1)                               # one U-turn counted
        self.assertFalse(cli.pos.forward or cli.pos.passes < 2)           # reversed after it

    def test_until_q_and_quit(self):
        cli, fake = quiet(), FakeEightFsm(v=0.5)
        cli.handle("0")
        drive(cli, fake, lambda: cli.pos.passes >= 3)
        self.assertTrue(cli.busy())                                       # 0 = no lap limit
        self.assertFalse(cli.handle("q"))
        self.assertEqual(cli.runs[0][4], "stopped (q)")
        self.assertFalse(cli.running())

    def test_bad_input_and_busy(self):
        said = []
        cli = EightCli(ask=None, say=said.append, live=False)
        cli.handle("abc")
        self.assertIn("number of laps", said[-1])
        cli.handle("2")
        cli.handle("3")                                                   # already driving
        self.assertIn("driving", said[-1])


class TestSettings(unittest.TestCase):
    def test_all_four_settings_go_into_the_run_copy(self):
        values = {(section, key): 0.111 * (i + 1) for i, (section, key, _) in enumerate(SETTINGS)}
        run_path = run_params(PARAMS_INI, values)
        try:
            for (section, key), value in values.items():
                self.assertAlmostEqual(ini_value(run_path, section, key), value)
        finally:
            os.remove(run_path)


if __name__ == "__main__":
    unittest.main()
