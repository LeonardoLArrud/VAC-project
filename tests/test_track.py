import unittest

from graph.track import Graph, figure_eight, from_edges, grid


class TestGraph(unittest.TestCase):
    def test_add_edge_is_undirected(self):
        g = Graph()
        g.add_edge("A", "B", 2)
        self.assertEqual(list(g.neighbors("A")), [("B", 2)])
        self.assertEqual(list(g.neighbors("B")), [("A", 2)])

    def test_negative_weight_rejected(self):
        with self.assertRaises(ValueError):
            Graph().add_edge("A", "B", -1)

    def test_block_and_unblock_both_directions(self):
        g = from_edges([("A", "B", 1)])
        g.block_edge("A", "B")
        self.assertTrue(g.is_blocked("B", "A"))
        self.assertEqual(list(g.neighbors("A")), [])
        self.assertEqual(list(g.neighbors("B")), [])
        g.unblock_edge("B", "A")
        self.assertEqual(list(g.neighbors("A")), [("B", 1)])

    def test_timed_block_expires(self):
        now = [0.0]
        g = from_edges([("A", "B", 1)], clock=lambda: now[0])
        g.block_edge("A", "B", duration=30)
        now[0] = 29.9
        self.assertTrue(g.is_blocked("A", "B"))
        now[0] = 30.0
        self.assertFalse(g.is_blocked("A", "B"))
        self.assertEqual(list(g.neighbors("A")), [("B", 1)])

    def test_default_block_is_permanent(self):
        now = [0.0]
        g = from_edges([("A", "B", 1)], clock=lambda: now[0])
        g.block_edge("A", "B")
        now[0] = 1e9
        self.assertTrue(g.is_blocked("A", "B"))

    def test_block_missing_edge_raises(self):
        g = from_edges([("A", "B", 1)])
        with self.assertRaises(KeyError):
            g.block_edge("A", "C")


class TestTracks(unittest.TestCase):
    def test_grid_5x5(self):
        g = grid(5)
        self.assertEqual(len(g.adj), 25)
        n_edges = sum(len(nbrs) for nbrs in g.adj.values()) // 2
        self.assertEqual(n_edges, 2 * 5 * 4)  # 20 horizontal + 20 vertical
        self.assertEqual(len(list(g.neighbors((0, 0)))), 2)  # corner
        self.assertEqual(len(list(g.neighbors((0, 2)))), 3)  # border
        self.assertEqual(len(list(g.neighbors((2, 2)))), 4)  # center

    def test_grid_spacing(self):
        g = grid(2, spacing=0.5)
        self.assertEqual(g.adj[(0, 0)][(0, 1)], 0.5)

    def test_figure_eight(self):
        g = figure_eight(nodes_per_loop=3)
        self.assertEqual(len(g.adj), 8)  # C1, C2 + 3 left + 3 right
        for node in g.adj:
            self.assertEqual(len(list(g.neighbors(node))), 2)  # a single ring
        # going straight through the crossing: left loop -> right loop
        self.assertEqual({v for v, _ in g.neighbors("C1")}, {"R2", "L0"})
        self.assertEqual({v for v, _ in g.neighbors("C2")}, {"L2", "R0"})

    def test_is_turn(self):
        g = figure_eight(3)
        self.assertTrue(g.is_turn("C2", "C1"))
        self.assertFalse(g.is_turn("C1", "L0"))

    def test_straight_ahead_on_grid(self):
        g = grid(5)
        self.assertEqual(g.straight_ahead((0, 0), (0, 1)), (0, 2))
        self.assertEqual(g.straight_ahead((1, 2), (2, 2)), (3, 2))
        self.assertIsNone(g.straight_ahead((0, 3), (0, 4)))  # border
        self.assertIsNone(g.straight_ahead(None, (0, 0)))  # no direction yet

    def test_straight_ahead_without_positions(self):
        g = from_edges([("A", "B", 1), ("B", "C", 1)])
        self.assertIsNone(g.straight_ahead("A", "B"))

    def test_figure_eight_positions(self):
        pos = figure_eight(3).pos
        self.assertEqual(pos["C1"], pos["C2"])  # same spot
        self.assertEqual(pos["L0"], (-1, 1))
        self.assertEqual(pos["L1"], (-2, 0))
        self.assertEqual(pos["R2"], (1, -1))

    def test_turn_edges_only_with_allow_turns(self):
        g = figure_eight(3)
        self.assertNotIn("C2", {v for v, _ in g.neighbors("C1")})
        self.assertIn(("C2", 0), list(g.neighbors("C1", allow_turns=True)))

    def test_from_edges(self):
        g = from_edges([(1, 2, 3.5), (2, 3, 1.0)])
        self.assertEqual(g.adj[2], {1: 3.5, 3: 1.0})


if __name__ == "__main__":
    unittest.main()
