"""cli.py without the robot: a fake FSM, the real navigators.Mission, no camera or serial."""
import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Robo_FSM", "pi"))

from rich.console import Console  # noqa: E402

from cli import FakeFsm, DeliveryCli, cli_hardware, grid_view, parse_node  # noqa: E402
from navigators import Mission  # noqa: E402


def quiet_cli(start=(0, 0), heading="E", n=5, g=1.0):
    said = []
    return DeliveryCli(start, heading, n, g, ask=None, say=said.append), said


def drive(mission):
    """Run a Mission node by node as the FSM would (no exits from the camera) until it stops."""
    while True:
        order, why = mission.on_node(None)
        if order == "STOP":
            return why


class TestParseNode(unittest.TestCase):
    def test_formats(self):
        for text in ("3 2", "3,2", "(3, 2)", "  3   2 "):
            self.assertEqual(parse_node(text, 5), (3, 2))

    def test_outside_or_garbage(self):
        for text in ("5 0", "-1 2", "3", "3 2 1", "abc", "3.5 2", ""):
            self.assertIsNone(parse_node(text, 5), text)


class TestDeliveries(unittest.TestCase):
    def test_destination_becomes_one_mission_command(self):
        cli, _ = quiet_cli()
        cli.handle("2 0")
        self.assertEqual(cli.command(), ("MISSION", (0, 0), "E", (2, 0), 5, 1.0))
        self.assertIsNone(cli.command())  # only once

    def test_arrival_then_next_mission_starts_there(self):
        cli, said = quiet_cli()
        fsm = SimpleNamespace(state="IDLE", nav=None, reason="")
        cli.attach(fsm)
        cli.handle("2 0")
        _, start, heading, goal, n, g = cli.command()
        fsm.state, fsm.nav = "NODE", Mission(start, heading, goal, n, g)
        self.assertIsNone(cli.command())  # still driving: no end yet
        fsm.reason = "STOP at the node: " + drive(fsm.nav)
        fsm.state = "IDLE"
        cli.command()
        self.assertEqual((cli.node, cli.heading, cli.goal), ((2, 0), "E", None))
        self.assertEqual(cli.deliveries[0][:2], ((2, 0), "arrived"))
        cli.handle("2 2")  # next one, from where it stopped
        self.assertEqual(cli.command(), ("MISSION", (2, 0), "E", (2, 2), 5, 1.0))

    def test_stopped_before_the_goal(self):
        cli, _ = quiet_cli()
        fsm = SimpleNamespace(state="IDLE", nav=None, reason="")
        cli.attach(fsm)
        cli.handle("4 4")
        cli.command()
        fsm.nav = SimpleNamespace(node=(1, 0), h_in=1)  # stopped at (1, 0) facing N
        fsm.reason = "STOP at the node: no path from (1, 0)"
        cli.command()
        self.assertEqual((cli.node, cli.heading), ((1, 0), "N"))
        self.assertEqual(cli.deliveries[0][1], fsm.reason)

    def test_no_new_destination_while_driving(self):
        cli, said = quiet_cli()
        cli.handle("2 0")
        cli.command()
        cli.handle("3 3")
        self.assertIsNone(cli.command())
        self.assertIn("driving to (2, 0)", said[-1])

    def test_invalid_or_same_node_is_refused(self):
        cli, said = quiet_cli()
        cli.handle("9 9")
        cli.handle("0 0")  # already there
        self.assertIsNone(cli.command())
        self.assertIn("already at", said[-1])

    def test_quit(self):
        cli, _ = quiet_cli()
        self.assertFalse(cli.handle("Q"))
        self.assertFalse(cli.running())
        self.assertEqual(cli.command(), "STOP")


class TestView(unittest.TestCase):
    def test_grid_shows_robot_and_goal(self):
        console = Console(record=True, width=60)
        console.print(grid_view(5, (0, 0), "N", goal=(4, 4)))
        text = console.export_text()
        self.assertIn("▲", text)
        self.assertIn("◎", text)
        self.assertEqual(text.count("·"), 23)  # 25 nodes - robot - goal


class TestCliHardware(unittest.TestCase):
    def test_subclass_gets_cli_commands_and_stops_after_quit(self):
        class Robot:                    # stands for RobotHardware / WebotsHardware
            def step(self):
                return True

            def command(self):
                return "START"          # e.g. a Webots key

            def read(self):             # like WebotsHardware.read(): calls self.command() itself
                return self.command()

        cli, _ = quiet_cli()
        hw = cli_hardware(Robot, cli)()
        cli.handle("1 0")
        self.assertEqual(hw.read()[0], "MISSION")   # the CLI's command goes first
        self.assertEqual(hw.read(), "START")        # then the class's own
        self.assertTrue(hw.step())
        cli.handle("q")
        self.assertFalse(hw.step())


class TestFakeFsm(unittest.TestCase):
    def test_drives_the_mission_node_by_node(self):
        cli, _ = quiet_cli()
        fsm = FakeFsm()
        cli.attach(fsm)
        cli.handle("2 0")
        fsm.step(cli.command(), t=0.0)  # leaves (0, 0)
        self.assertEqual(fsm.state, "NODE")
        fsm.step(cli.command(), t=1.0)  # at (1, 0)
        fsm.step(cli.command(), t=2.0)  # at (2, 0): the goal
        self.assertEqual(fsm.state, "IDLE")
        cli.command()                   # the CLI notices the arrival
        self.assertEqual(cli.deliveries[0][:2], ((2, 0), "arrived"))


if __name__ == "__main__":
    unittest.main()
