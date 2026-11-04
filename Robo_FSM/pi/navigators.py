"""Navigators: where to go at a node and what to do when blocked."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))   # Robo_FSM/graph/: the first one going up (pi/.. or <repo>/Robo_FSM)
ROOTS = (os.path.join(HERE, *[".."] * k, sub) for k in range(6) for sub in ("", "Robo_FSM"))
sys.path.insert(0, next(p for p in ROOTS if os.path.isdir(os.path.join(p, "graph"))))
from graph.path import dijkstra
from graph.track import grid

SIDE = {"S": "STRAIGHT", "L": "LEFT", "R": "RIGHT"}
STEP = [(1, 0), (0, 1), (-1, 0), (0, -1)]          # E, N, W, S
ORDER = ["STRAIGHT", "LEFT", "U_TURN", "RIGHT"]    # by (d - h) % 4
EXIT = {0: "S", 1: "L", 3: "R"}


class Navigator:  # every method answers (order, reason)
    def on_node(self, exits, s_node=0.0):
        return "STOP", "no navigator"

    def on_blocked(self, s, d):  # U_TURN, WAIT or STOP
        return "STOP", "blocked"

    def on_fault(self, kind):    # GO_ON or STOP
        return "GO_ON", kind

    def on_exit_blocked(self, exits):   # sonar: the exit just chosen is blocked. GO_ON (turn anyway) or a new order
        return "GO_ON", "exit blocked: not handled"

    def on_clear(self):
        pass

    def prior(self):             # (s_next, exits) or None
        return None


class Explorer(Navigator):  # no goal (the 8)
    u_turns = 0

    def on_node(self, exits, s_node=0.0):   # S, else the only exit, else STOP
        e = {"S"} if "S" in exits else set(exits)
        return (SIDE[e.pop()], "explorer") if len(e) == 1 else ("STOP", f"explorer: no single exit in {sorted(exits)}")

    def on_blocked(self, s, d):  # U-turn every time it is still blocked: it tries again, never gives up
        self.u_turns += 1
        return "U_TURN", f"still blocked: U-turn ({self.u_turns} in a row)"

    def on_clear(self):
        self.u_turns = 0


class OrderList(Navigator):
    def __init__(self, orders):
        self.orders = list(orders)

    def on_node(self, exits, s_node=0.0):
        return (self.orders.pop(0), "order list") if self.orders else ("STOP", "order list empty")


class Mission(Navigator):  # goal on the grid, path recomputed at every node
    def __init__(self, start, heading, goal, n=5, g=0.50):
        self.map, self.n, self.g = grid(n, g), n, g
        self.to, self.h, self.goal = tuple(start), "ENWS".index(heading), tuple(goal)
        self.node = None
        self.start, self.path, self.closed = tuple(start), [], []

    def _id(self, ij):           # (i, j) -> graph.track (row, col), row 0 on top
        return (self.n - 1 - ij[1], ij[0])

    def _step(self, ij, d):
        return (ij[0] + STEP[d][0], ij[1] + STEP[d][1])

    def _exits(self, node, h):
        nbrs = self.map.adj.get(self._id(node), {})
        return {EXIT[(d - h) % 4] for d in range(4) if (d - h) % 4 in EXIT and self._id(self._step(node, d)) in nbrs}

    def on_node(self, exits, s_node=0.0):   # exits None: from the map
        node, h_in = self.to, self.h
        if exits is not None and not set(exits) <= self._exits(node, h_in):
            return "STOP", f"pose lost at {node}: saw {sorted(exits)}, map has {sorted(self._exits(node, h_in))}"
        self.node, self.h_in, self.s_at, self.s_next = node, h_in, s_node, s_node + self.g
        if node == self.goal:
            self.path = [node]
            return "STOP", f"arrived at {node}"
        while True:
            path, _ = dijkstra(self.map, self._id(node), self._id(self.goal))
            if not path:
                return "STOP", f"no path from {node}"
            (x0, y0), (x1, y1) = self.map.pos[path[0]], self.map.pos[path[1]]
            d = STEP.index((x1 - x0, y1 - y0))
            rel = (d - h_in) % 4
            if exits is None or rel == 2 or EXIT[rel] in exits:
                break
            self.map.block_edge(path[0], path[1])   # exit not there: drop the edge
            self.closed.append((node, self._step(node, d)))
        self.h, self.to = d, self._step(node, d)
        self.path = [(c, self.n - 1 - r) for r, c in path]
        return ORDER[rel], f"mission: at {node}, next {self.to}"

    def on_blocked(self, s, d):  # drop the edge of the obstacle, back to the node behind
        prev, passed = self._step(self.node, (self.h_in + 2) % 4), s >= self.s_at
        edge = (self.node, self.to) if s + d > self.s_at else (prev, self.node)
        self.map.block_edge(self._id(edge[0]), self._id(edge[1]))
        self.closed.append(edge)
        self.path = []
        back, s_back = (self.node, self.s_at) if passed else (prev, self.s_at - self.g)
        self.to, self.h = back, ((self.h if passed else self.h_in) + 2) % 4
        self.s_next = 2 * s - s_back            # the node behind is s - s_back away
        return "U_TURN", f"edge {edge[0]}-{edge[1]} blocked: back to {back}"

    def on_exit_blocked(self, exits):   # sonar at the node centre: drop the exit just chosen, choose again from here
        edge = (self.node, self.to)
        self.map.block_edge(self._id(edge[0]), self._id(edge[1]))
        self.closed.append(edge)
        self.to, self.h = self.node, self.h_in
        return self.on_node(exits, self.s_at)

    def on_fault(self, kind):    # node missed: counted as passed
        if kind != "node missed":
            return "GO_ON", kind
        self.node, self.h_in, self.s_at = self.to, self.h, self.s_next
        self.to, self.s_next = self._step(self.to, self.h), self.s_next + self.g
        if self._id(self.to) not in self.map.adj:
            return "STOP", f"pose lost: node {self.node} missed at the border"
        return "GO_ON", f"node {self.node} missed, counted"

    def prior(self):
        return (self.s_next, self._exits(self.to, self.h)) if self.node else None

    def view(self):              # for controllers/mission_view
        return {"n": self.n, "g": self.g, "start": self.start, "goal": self.goal, "path": self.path, "closed": self.closed}
