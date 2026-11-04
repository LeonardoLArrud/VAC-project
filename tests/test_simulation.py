import unittest

from graph.simulation import Simulation, edges, nearest_edge, nearest_node
from graph.track import figure_eight, grid


class TestSimulationStep(unittest.TestCase):
    def test_step_moves_and_logs_maneuver(self):
        sim = Simulation(grid(5), (1, 0))
        sim.set_goal((1, 2))
        text = sim.step()
        self.assertEqual(sim.nav.current, (1, 1))
        self.assertIn("(1, 0) -> (1, 1)", text)

    def test_delivery_then_back_to_start_and_wait(self):
        sim = Simulation(grid(5), (0, 0))
        sim.set_goal((0, 1))
        self.assertEqual(sim.step(), "follow at (0, 0) -> (0, 1)   DELIVERED, going back to start")
        self.assertEqual(sim.nav.goal, (0, 0))
        self.assertEqual(sim.step(), "u_turn at (0, 1) -> (0, 0)")
        for _ in range(3):  # stays at the start, does not move
            self.assertEqual(sim.step(), "waiting at start (0, 0) for the next delivery")
            self.assertEqual(sim.nav.current, (0, 0))

    def test_new_delivery_while_waiting(self):
        sim = Simulation(grid(5), (0, 0))
        sim.set_goal((0, 1))
        sim.step()  # delivered
        sim.step()  # back at the start
        sim.set_goal((1, 0))
        self.assertEqual(sim.step(), "follow at (0, 0) -> (1, 0)   DELIVERED, going back to start")

    def test_moved_car_returns_to_its_new_start(self):
        sim = Simulation(grid(5), (1, 1))
        sim.move_car((0, 0))
        sim.set_goal((0, 1))
        self.assertIn("DELIVERED", sim.step())
        self.assertEqual(sim.nav.goal, (0, 0))  # not (1, 1)
        sim.step()
        self.assertEqual(sim.step(), "waiting at start (0, 0) for the next delivery")

    def test_no_goal_keeps_cruising_from_the_start(self):
        sim = Simulation(grid(5), (1, 1))
        sim.step()
        sim.step()
        self.assertNotEqual(sim.nav.current, (1, 1))  # no "waiting" without a delivery

    def test_hidden_obstacle_found_by_sensor(self):
        now = [0.0]
        sim = Simulation(grid(5, clock=lambda: now[0]), (0, 0))
        sim.set_goal((0, 2))
        sim.toggle_dynamic((0, 0), (0, 1))
        self.assertFalse(sim.graph.is_blocked((0, 0), (0, 1)))  # car doesn't know yet

        # step 1: drives into the edge, sees the obstacle, turns around
        self.assertEqual(sim.step(), "obstacle seen on (0, 0)-(0, 1): turning around")
        self.assertTrue(sim.graph.is_blocked((0, 0), (0, 1)))
        self.assertEqual(sim.car_position(), (0.25, 0.0))  # stopped in front of it
        # step 2: back at the node it came from, then goes around
        self.assertEqual(sim.step(), "back at (0, 0) after turning around")
        self.assertEqual(sim.car_position(), (0, 0))
        self.assertEqual(sim.nav.next_node(), (1, 0))

        now[0] = 1000  # block expired
        self.assertFalse(sim.graph.is_blocked((0, 0), (0, 1)))

    def test_obstacle_after_crossing_on_the_eight(self):
        sim = Simulation(figure_eight(3), "L0")  # no goal: cruising
        sim.step()  # L0 -> C1
        sim.toggle_dynamic("C1", "R2")
        self.assertEqual(sim.step(), "obstacle seen on C1-R2: turning around")
        self.assertEqual(sim.step(), "back at C1 after turning around")
        # already turned around on the edge: straight back through the crossing
        self.assertEqual(sim.step(), "straight at C1 -> L0")

    def test_obstacle_after_crossing_with_goal_takes_sharp_turn(self):
        sim = Simulation(figure_eight(3), "L0")
        sim.step()  # L0 -> C1 (cruising)
        sim.set_goal("R0")  # normally C1 -> R2 -> R1 -> R0
        sim.toggle_dynamic("C1", "R2")
        sim.step()  # sees it, turns around
        sim.step()  # back at C1, coming from R2
        # heading up-left (from R2), sharp turn to R0 (up-right) = right
        self.assertEqual(sim.step(), "right at C2 -> R0   DELIVERED, going back to start")

    def test_grid_visits_all_nodes_after_obstacle(self):
        sim = Simulation(grid(5), (1, 1))
        sim.toggle_dynamic((0, 3), (0, 2))  # on the border, where the car laps
        for _ in range(16):
            sim.step()
        self.assertTrue(sim.log[-1].startswith("obstacle seen"))
        start = len(sim.nav.history) - 1
        for _ in range(26):
            sim.step()
        self.assertEqual(len(set(sim.nav.history[start:])), 25)
        self.assertEqual(sim.nav.to_visit, set())

    def test_obstacle_next_to_crossing_does_not_trap_the_car(self):
        for edge in [("C1", "R2"), ("C1", "L0"), ("C2", "L2"), ("C2", "R0")]:
            with self.subTest(edge=edge):
                sim = Simulation(figure_eight(3, clock=lambda: 0.0), "L0")  # stays blocked
                sim.toggle_dynamic(*edge)
                for _ in range(60):
                    sim.step()
                # long after the visit-all tour, it still goes round the whole 8
                self.assertEqual(len(set(sim.nav.history[-24:])), 8)

    def test_toggle_static_blocks_forever_and_replans(self):
        sim = Simulation(grid(5), (0, 0))
        sim.set_goal((0, 1))
        sim.toggle_static((0, 0), (0, 1))
        self.assertTrue(sim.graph.is_blocked((0, 0), (0, 1)))
        self.assertEqual(sim.nav.distance, 3)
        sim.toggle_static((0, 0), (0, 1))
        self.assertFalse(sim.graph.is_blocked((0, 0), (0, 1)))
        self.assertEqual(sim.nav.distance, 1)

    def test_stuck(self):
        sim = Simulation(grid(5), (0, 0))
        sim.toggle_static((0, 0), (0, 1))
        sim.toggle_static((0, 0), (1, 0))
        self.assertIn("stuck", sim.step())

    def test_move_car_keeps_goal(self):
        sim = Simulation(grid(5), (0, 0))
        sim.set_goal((4, 4))
        sim.move_car((3, 4))
        self.assertEqual(sim.nav.current, (3, 4))
        self.assertEqual(sim.nav.goal, (4, 4))
        self.assertEqual(sim.nav.history, [(3, 4)])


class TestGeometry(unittest.TestCase):
    def test_edges_listed_once_without_turn_edge(self):
        g = figure_eight(3)
        found = [frozenset(e) for e in edges(g)]
        self.assertEqual(len(found), 8)  # ring of 8 nodes
        self.assertEqual(len(set(found)), 8)
        self.assertNotIn(frozenset(("C1", "C2")), found)
        self.assertEqual(len(list(edges(grid(5)))), 40)

    def test_nearest_node(self):
        node, dist = nearest_node(grid(5), 2.1, -0.9)  # near (row 1, col 2)
        self.assertEqual(node, (1, 2))
        self.assertAlmostEqual(dist, (0.1**2 + 0.1**2) ** 0.5)

    def test_nearest_edge(self):
        (u, v), dist = nearest_edge(grid(5), 0.5, -0.1)  # just below the top-left edge
        self.assertEqual(frozenset((u, v)), frozenset(((0, 0), (0, 1))))
        self.assertAlmostEqual(dist, 0.1)


if __name__ == "__main__":
    unittest.main()
