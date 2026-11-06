import unittest

from graph.navigation import Navigator
from graph.track import figure_eight, grid


class TestNavigator(unittest.TestCase):
    def test_reaches_goal_without_obstacles(self):
        nav = Navigator(grid(5), (0, 0), (2, 2))
        steps = 0
        while not nav.arrived():
            nav.advance()
            steps += 1
        self.assertEqual(steps, 4)

    def test_static_obstacle_blocked_before_start(self):
        g = grid(5)
        g.block_edge((0, 0), (0, 1))
        nav = Navigator(g, (0, 0), (0, 1))
        self.assertEqual(nav.distance, 3)  # goes around: down, right, up
        self.assertEqual(nav.next_node(), (1, 0))

    def test_dynamic_obstacle_triggers_replan(self):
        g = figure_eight(3)
        nav = Navigator(g, "L0", "C1")
        self.assertEqual(nav.next_node(), "C1")
        nav.obstacle_ahead()
        self.assertTrue(g.is_blocked("L0", "C1"))
        self.assertEqual(nav.path, ["L0", "L1", "L2", "C2", "C1"])  # sharp turn
        while not nav.arrived():
            nav.advance()
        self.assertEqual(nav.current, "C1")

    def test_freed_edge_is_used_again(self):
        now = [0.0]
        g = figure_eight(4, clock=lambda: now[0])
        nav = Navigator(g, "L0", "C1")
        nav.obstacle_ahead(duration=30)  # blocks L0-C1
        self.assertEqual(nav.distance, 4)  # L0-L1-L2-L3-C2, sharp turn to C1

        now[0] = 31  # obstacle gone by the time the car is back at L0
        nav.advance()  # back at L0 after turning around on the edge
        self.assertEqual(nav.path, ["L0", "C1"])  # the freed edge is used again

    def test_sharp_turn_only_after_obstacle(self):
        nav = Navigator(figure_eight(4), "L0", "L3")
        # the turn L0-C1-C2-L3 would be shorter, but there is no obstacle yet
        self.assertEqual(nav.path, ["L0", "L1", "L2", "L3"])
        nav.obstacle_ahead()  # blocks L0-L1
        self.assertEqual(nav.path, ["L0", "C1", "C2", "L3"])

    def test_no_path_left(self):
        g = figure_eight(3)
        nav = Navigator(g, "L0", "C1")
        nav.obstacle_ahead()  # blocks L0-C1, turns around on the edge
        nav.advance()  # back at L0
        nav.advance()  # now at L1 (going the other way round)
        nav.obstacle_ahead()  # blocks L1-L2: cut off from C1
        nav.advance()  # back at L1
        self.assertEqual(nav.path, [])
        self.assertEqual(nav.next_node(), "L0")  # keeps driving meanwhile

    def test_turn_around_on_the_edge(self):
        nav = Navigator(grid(5), (0, 0), (0, 2))
        nav.obstacle_ahead()  # sees an obstacle on (0,0)-(0,1)
        self.assertEqual(nav.next_node(), (0, 0))  # drives back to where it came from
        nav.advance()  # back at (0, 0)
        self.assertEqual(nav.previous, (0, 1))  # arrives from the blocked side
        self.assertEqual(nav.history, [(0, 0), (0, 0)])
        self.assertEqual(nav.next_node(), (1, 0))  # then goes around


class TestFacing(unittest.TestCase):
    def test_cruise_starts_the_way_the_car_points(self):
        nav = Navigator(grid(5), (1, 1), facing=(2, 1))  # no goal, pointing down
        self.assertEqual(nav.next_node(), (2, 1))
        nav.advance()
        self.assertEqual(nav.next_node(), (3, 1))  # then straight on

    def test_without_facing_unchanged(self):
        nav = Navigator(grid(5), (1, 1))
        self.assertIsNone(nav.facing)
        self.assertEqual(nav.next_node(), (0, 1))  # first free neighbor, as before

    def test_goal_still_third_argument(self):
        nav = Navigator(grid(5), (1, 1), (1, 3))
        self.assertEqual(nav.goal, (1, 3))


class TestStart(unittest.TestCase):
    def test_start_stays_while_the_car_moves(self):
        nav = Navigator(grid(5), (1, 1), (1, 3))
        nav.advance()
        nav.advance()
        self.assertEqual(nav.current, (1, 3))
        self.assertEqual(nav.start, (1, 1))  # current changes, start does not


class TestHistory(unittest.TestCase):
    def test_records_trip(self):
        nav = Navigator(grid(5), (0, 0), (2, 0))
        while not nav.arrived():
            nav.advance()
        self.assertEqual(nav.history, [(0, 0), (1, 0), (2, 0)])

    def test_records_detour_and_sharp_turn(self):
        nav = Navigator(figure_eight(3), "L0", "L2")
        nav.obstacle_ahead()  # blocks L0-L1 -> sharp turn at the crossing
        while not nav.arrived():
            nav.advance()
        # L0 twice: the car drove back to it after turning around on L0-L1
        self.assertEqual(nav.history, ["L0", "L0", "C1", "C2", "L2"])

    def test_keeps_recording_after_goal(self):
        nav = Navigator(grid(5), (0, 0), (0, 1))
        nav.advance()  # arrives
        nav.advance()  # keeps going straight
        self.assertEqual(nav.history, [(0, 0), (0, 1), (0, 2)])


class TestSharpTurnStep(unittest.TestCase):
    def test_crossing_turn_needs_no_advance(self):
        nav = Navigator(figure_eight(3), "L0", "L2")
        nav.obstacle_ahead()  # blocks L0-L1 -> L0, C1, C2, L2
        nav.advance()  # back at L0
        nav.advance()  # car reaches the crossing and turns on the spot
        self.assertEqual(nav.current, "C2")
        self.assertEqual(nav.next_node(), "L2")


class TestCruise(unittest.TestCase):
    def drive(self, nav, steps):
        visited = []
        for _ in range(steps):
            nav.advance()
            visited.append(nav.current)
        return visited

    def test_laps_the_eight_straight_through_crossing(self):
        nav = Navigator(figure_eight(3), "L0")
        visited = self.drive(nav, 16)
        self.assertEqual(visited[:8], visited[8:])  # one lap = 8 nodes, repeated
        self.assertEqual(set(visited), {"C1", "C2", "L0", "L1", "L2", "R0", "R1", "R2"})
        for u, v in zip(visited, visited[1:]):
            self.assertNotEqual({u, v}, {"C1", "C2"})  # never the sharp turn

    def test_grid_goes_straight_then_turns_at_border(self):
        nav = Navigator(grid(5), (0, 0))
        visited = self.drive(nav, 5)
        self.assertEqual(visited[:4], [(0, 1), (0, 2), (0, 3), (0, 4)])
        self.assertEqual(visited[4], (1, 4))

    def cruising_at(self, g, start, node):
        """Navigator cruising (no goal), standing at node, coming from start."""
        nav = Navigator(g, start, node)
        nav.advance()
        nav.set_goal(None)
        return nav

    def test_u_turn_when_blocked_ahead(self):
        nav = self.cruising_at(figure_eight(3), "L1", "L2")
        nav.obstacle_ahead()  # sees an obstacle on L2-C2: turns around on the edge
        self.assertEqual(nav.next_node(), "L2")  # drives back to L2
        nav.advance()  # back at L2
        self.assertEqual(nav.next_node(), "L1")

    def test_stuck_when_everything_blocked(self):
        nav = self.cruising_at(figure_eight(3), "L1", "L2")
        nav.obstacle_ahead()  # blocks L2-C2
        nav.advance()  # back at L2
        nav.obstacle_ahead()  # blocks L2-L1
        nav.advance()  # back at L2 again
        self.assertIsNone(nav.next_node())
        with self.assertRaises(RuntimeError):
            nav.advance()

    def test_obstacle_after_crossing_turns_back_on_the_edge(self):
        nav = self.cruising_at(figure_eight(3), "L0", "C1")
        nav.obstacle_ahead()  # sees an obstacle on C1-R2
        nav.advance()  # back at C1, coming from R2
        self.assertEqual(nav.previous, "R2")
        self.assertEqual(nav.next_node(), "L0")  # straight on: it already turned around

    def test_sharp_turn_instead_of_u_turn_static(self):
        g = figure_eight(3)
        nav = self.cruising_at(g, "L0", "C1")
        g.block_edge("C1", "R2")
        nav.replan()
        self.assertEqual(nav.current, "C2")
        self.assertEqual(nav.next_node(), "L2")

    def test_chooses_branch_visited_longest_ago(self):
        nav = Navigator(grid(5), (1, 0), (1, 1))  # at (1,1) heading right
        nav.advance()
        nav.set_goal(None)
        nav.graph.block_edge((1, 1), (1, 2))  # no straight on: up or down
        nav.last_visit[(0, 1)] = 5  # up was visited more recently than down
        nav.last_visit[(2, 1)] = 2
        self.assertEqual(nav.next_node(), (2, 1))

    def test_keeps_going_after_goal(self):
        nav = Navigator(grid(5), (0, 0), (0, 2))
        self.drive(nav, 2)
        self.assertTrue(nav.arrived())
        self.assertEqual(nav.next_node(), (0, 3))

    def test_set_goal_while_cruising(self):
        nav = Navigator(grid(5), (0, 0))
        nav.advance()
        nav.set_goal((2, 1))
        self.assertEqual((nav.path[0], nav.path[-1]), (nav.current, (2, 1)))
        self.assertEqual(nav.distance, 2)


class TestVisitAll(unittest.TestCase):
    def cruising_at(self, g, start, node):
        nav = Navigator(g, start, node)
        nav.advance()
        nav.set_goal(None)
        return nav

    def test_obstacle_without_goal_starts_visit_all(self):
        g = figure_eight(3)
        nav = self.cruising_at(g, "C2", "L2")
        nav.obstacle_ahead()  # obstacle on L2-L1
        self.assertEqual(nav.to_visit, set(g.adj) - {"L2"})

    def test_eight_visits_every_node_without_going_back(self):
        nav = self.cruising_at(figure_eight(3), "C2", "L2")
        nav.obstacle_ahead()  # obstacle on L2-L1: turns around
        nav.advance()  # back at L2
        visited = []
        for _ in range(7):
            nav.advance()
            visited.append(nav.current)
        self.assertEqual(visited, ["C2", "R0", "R1", "R2", "C1", "L0", "L1"])
        self.assertEqual(nav.to_visit, set())  # done: back to just driving

    def test_obstacle_with_goal_does_not_start_visit_all(self):
        nav = Navigator(grid(5), (0, 0), (0, 2))
        nav.obstacle_ahead()
        self.assertEqual(nav.to_visit, set())

    def test_new_goal_cancels_visit_all(self):
        nav = self.cruising_at(figure_eight(3), "C2", "L2")
        nav.obstacle_ahead()
        nav.advance()
        nav.set_goal("R1")
        self.assertEqual(nav.to_visit, set())
        self.assertEqual(nav.path[-1], "R1")

    def test_unreachable_nodes_do_not_stop_the_car(self):
        g = figure_eight(3)
        g.block_edge("L0", "C1")  # with L1-L2 below, L0 and L1 are cut off
        nav = self.cruising_at(g, "L0", "L1")
        nav.obstacle_ahead()  # obstacle on L1-L2
        for _ in range(5):
            nav.advance()  # shuttles between L0 and L1, no crash
        self.assertIn(nav.current, ("L0", "L1"))
        self.assertTrue(nav.to_visit)  # the rest cannot be reached for now


class TestSharpTurnRule(unittest.TestCase):
    def test_turns_allowed_only_while_something_is_blocked(self):
        now = [0.0]
        g = figure_eight(4, clock=lambda: now[0])
        nav = Navigator(g, "L0", "L3")
        self.assertFalse(nav.allow_turns)
        g.block_edge("R0", "R1", duration=30)  # anywhere on the track
        self.assertTrue(nav.allow_turns)
        now[0] = 31
        self.assertFalse(nav.allow_turns)

    def test_static_obstacle_with_goal_uses_sharp_turn(self):
        g = figure_eight(3)
        nav = Navigator(g, "L0", "C1")
        nav.advance()  # at C1, coming from L0
        nav.set_goal("R1")
        self.assertEqual(nav.path, ["C1", "R2", "R1"])
        g.block_edge("C1", "R2")  # static obstacle in front
        nav.replan()
        self.assertEqual(nav.current, "C2")  # sharp turn at the crossing
        self.assertEqual(nav.path, ["C2", "R0", "R1"])


if __name__ == "__main__":
    unittest.main()
