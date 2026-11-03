import unittest

from graph.maneuver import (
    FOLLOW,
    LEFT,
    RIGHT,
    STRAIGHT,
    U_TURN,
    maneuver,
    maneuvers,
)
from graph.navigation import Navigator
from graph.path import dijkstra
from graph.track import figure_eight, from_edges, grid


class TestManeuverGrid(unittest.TestCase):
    def setUp(self):
        self.g = grid(5)  # row 0 at the top

    def test_straight(self):
        self.assertEqual(maneuver(self.g, (0, 0), (0, 1), (0, 2)), STRAIGHT)

    def test_left_and_right(self):
        # heading right along row 1, then up (row 0) or down (row 2)
        self.assertEqual(maneuver(self.g, (1, 0), (1, 1), (0, 1)), LEFT)
        self.assertEqual(maneuver(self.g, (1, 0), (1, 1), (2, 1)), RIGHT)

    def test_u_turn(self):
        self.assertEqual(maneuver(self.g, (0, 1), (1, 1), (0, 1)), U_TURN)

    def test_corner_has_no_choice(self):
        self.assertEqual(maneuver(self.g, (0, 1), (0, 0), (1, 0)), FOLLOW)

    def test_unknown_heading_at_start(self):
        self.assertEqual(maneuver(self.g, None, (1, 1), (1, 2)), FOLLOW)


class TestManeuverEight(unittest.TestCase):
    def setUp(self):
        self.g = figure_eight(3)

    def test_straight_through_crossing(self):
        self.assertEqual(maneuver(self.g, "L2", "C2", "R0"), STRAIGHT)
        self.assertEqual(maneuver(self.g, "R2", "C1", "L0"), STRAIGHT)

    def test_loop_node_has_no_choice(self):
        self.assertEqual(maneuver(self.g, "L0", "L1", "L2"), FOLLOW)

    def test_sharp_turn_at_crossing(self):
        # coming down-right from L0, leaving down-left to L2
        self.assertEqual(maneuver(self.g, "L0", "C2", "L2"), RIGHT)

    def test_navigator_gives_the_sharp_turn(self):
        nav = Navigator(self.g, "L0", "L2")
        nav.obstacle_ahead()  # blocks L0-L1 -> L0, C1, C2, L2
        nav.advance()  # back at L0 after turning around on the edge
        nav.advance()  # at the crossing (C1 -> C2 skipped)
        self.assertEqual(
            maneuver(nav.graph, nav.previous, nav.current, nav.next_node()), RIGHT
        )


class TestManeuverAfterTurningAround(unittest.TestCase):
    def test_direction_is_reversed_after_coming_back(self):
        nav = Navigator(grid(5), (1, 1), (1, 2))  # heading right
        nav.obstacle_ahead()  # obstacle on (1,1)-(1,2): turns around on the edge
        nav.advance()  # back at (1, 1), now heading LEFT
        nav.set_goal((0, 1))  # up
        # heading left, going up = turning RIGHT (it would be LEFT before turning around)
        self.assertEqual(
            maneuver(nav.graph, nav.previous, nav.current, nav.next_node()), RIGHT
        )


class TestManeuvers(unittest.TestCase):
    def test_sequence_on_grid(self):
        path = [(1, 0), (1, 1), (1, 2), (0, 2)]
        self.assertEqual(maneuvers(grid(5), path), [FOLLOW, STRAIGHT, LEFT])

    def test_one_per_node_except_goal(self):
        g = grid(5)
        path, _ = dijkstra(g, (0, 0), (4, 4))
        self.assertEqual(len(maneuvers(g, path)), len(path) - 1)

    def test_sharp_turn_edge_is_not_a_step(self):
        path = ["L0", "C1", "C2", "L2"]
        self.assertEqual(maneuvers(figure_eight(3), path, previous="L1"), [FOLLOW, RIGHT])

    def test_empty_path(self):
        self.assertEqual(maneuvers(grid(5), []), [])


class TestNoPositions(unittest.TestCase):
    def test_intersection_without_positions_raises(self):
        g = from_edges([("A", "O", 1), ("B", "O", 1), ("C", "O", 1)])
        with self.assertRaises(ValueError):
            maneuver(g, "A", "O", "B")


if __name__ == "__main__":
    unittest.main()
