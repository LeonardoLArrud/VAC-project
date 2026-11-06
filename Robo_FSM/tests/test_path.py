import unittest
from math import inf

from graph.path import dijkstra
from graph.track import figure_eight, from_edges, grid


class TestDijkstra(unittest.TestCase):
    def test_start_equals_goal(self):
        self.assertEqual(dijkstra(grid(5), (0, 0), (0, 0)), ([(0, 0)], 0))

    def test_grid_corner_to_corner(self):
        path, dist = dijkstra(grid(5), (0, 0), (4, 4))
        self.assertEqual(dist, 8)  # Manhattan distance
        self.assertEqual(len(path), 9)
        self.assertEqual((path[0], path[-1]), ((0, 0), (4, 4)))

    def test_path_is_made_of_real_edges(self):
        g = grid(5)
        path, dist = dijkstra(g, (0, 0), (3, 4))
        self.assertEqual(sum(g.adj[u][v] for u, v in zip(path, path[1:])), dist)

    def test_figure_eight_crosses_center(self):
        path, dist = dijkstra(figure_eight(3), "L1", "R1")
        self.assertEqual((path[0], path[-1]), ("L1", "R1"))
        self.assertTrue("C1" in path or "C2" in path)
        self.assertEqual(dist, 4)  # e.g. L1 -> L2 -> C2 -> R0 -> R1

    def test_figure_eight_cannot_turn_back_at_crossing(self):
        g = figure_eight(3)
        g.block_edge("L1", "L2")
        path, dist = dijkstra(g, "L0", "L2")
        # must go around the whole right loop, not L0 -> crossing -> L2
        self.assertEqual(path, ["L0", "C1", "R2", "R1", "R0", "C2", "L2"])
        self.assertEqual(dist, 6)

    def test_figure_eight_sharp_turn_when_allowed(self):
        g = figure_eight(3)
        g.block_edge("L1", "L2")
        path, dist = dijkstra(g, "L0", "L2", allow_turns=True)
        self.assertEqual(path, ["L0", "C1", "C2", "L2"])  # turn at the crossing
        self.assertEqual(dist, 2)

    def test_weighted_prefers_cheaper_longer_route(self):
        g = from_edges([("A", "B", 10), ("A", "C", 1), ("C", "D", 1), ("D", "B", 1)])
        self.assertEqual(dijkstra(g, "A", "B"), (["A", "C", "D", "B"], 3))

    def test_avoids_blocked_edge(self):
        g = from_edges([("A", "B", 1), ("A", "C", 1), ("C", "B", 1)])
        g.block_edge("A", "B")
        self.assertEqual(dijkstra(g, "A", "B"), (["A", "C", "B"], 2))

    def test_unreachable_returns_empty(self):
        g = from_edges([("A", "B", 1)])
        g.block_edge("A", "B")
        self.assertEqual(dijkstra(g, "A", "B"), ([], inf))

    def test_unknown_node_raises(self):
        with self.assertRaises(KeyError):
            dijkstra(grid(5), (0, 0), (9, 9))

    def test_mixed_node_types(self):
        g = from_edges([(0, "A", 1), (0, (1, 1), 1), ("A", "Z", 1), ((1, 1), "Z", 1)])
        _, dist = dijkstra(g, 0, "Z")
        self.assertEqual(dist, 2)


if __name__ == "__main__":
    unittest.main()
