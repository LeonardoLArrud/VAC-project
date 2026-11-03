import time
from math import cos, inf, pi, sin


class Graph:
    """Undirected weighted graph of the track. Edges can be blocked by obstacles,
    forever (static) or for a limited time (dynamic)."""

    def __init__(self, clock=time.monotonic):
        self.adj = {}  # node -> {neighbor: weight}
        self.blocked_until = {}  # frozenset({u, v}) -> time when it is free again
        self.turns = set()  # frozenset({u, v}) of sharp-turn edges
        self.pos = {}  # node -> (x, y): x to the right, y up (used for turns)
        self.clock = clock  # replaceable, so tests can control time

    def add_edge(self, u, v, weight=1.0, turn=False):
        """turn=True: a sharp turn, only usable when neighbors() allows turns."""
        if weight < 0:
            raise ValueError("Dijkstra needs non-negative weights")
        self.adj.setdefault(u, {})[v] = weight
        self.adj.setdefault(v, {})[u] = weight
        if turn:
            self.turns.add(frozenset((u, v)))

    def neighbors(self, u, allow_turns=False):
        """Yield (neighbor, weight) for every usable edge of u:
        not blocked, and not a sharp turn unless allow_turns."""
        for v, w in self.adj[u].items():
            if self.is_blocked(u, v):
                continue
            if self.is_turn(u, v) and not allow_turns:
                continue
            yield v, w

    def is_turn(self, u, v):
        return frozenset((u, v)) in self.turns

    def straight_ahead(self, prev, cur):
        """Neighbor of cur in the direction prev -> cur, or None (needs positions)."""
        if prev not in self.pos or cur not in self.pos:
            return None
        target = tuple(2 * c - p for c, p in zip(self.pos[cur], self.pos[prev]))
        for v in self.adj[cur]:
            if self.pos.get(v) == target:
                return v
        return None

    def block_edge(self, u, v, duration=inf):
        """Block an edge for `duration` seconds (default: forever)."""
        if v not in self.adj.get(u, {}):
            raise KeyError(f"no edge {u} - {v}")
        self.blocked_until[frozenset((u, v))] = self.clock() + duration

    def unblock_edge(self, u, v):
        self.blocked_until.pop(frozenset((u, v)), None)

    def is_blocked(self, u, v):
        return self.blocked_until.get(frozenset((u, v)), -inf) > self.clock()

    def has_blocked(self):
        """True while at least one edge is blocked."""
        now = self.clock()
        return any(until > now for until in self.blocked_until.values())


def grid(n=5, spacing=1.0, clock=time.monotonic):
    """n x n grid. Nodes are (row, col) tuples, row 0 at the top."""
    g = Graph(clock)
    for r in range(n):
        for c in range(n):
            g.pos[(r, c)] = (c, -r)
            if c + 1 < n:
                g.add_edge((r, c), (r, c + 1), spacing)
            if r + 1 < n:
                g.add_edge((r, c), (r + 1, c), spacing)
    return g


def figure_eight(nodes_per_loop=3, spacing=1.0, clock=time.monotonic):
    """Figure 8 where the car normally goes straight through the crossing.
    The crossing is split into C1 and C2 (same place, one node per line through it),
    so the 8 is a single ring: C1-L0-...-Ln-C2-R0-...-Rn-C1.
    The turn edge C1-C2 (length 0) is the sharp turn at the crossing.

    Positions (shape of huit.jpg): crossing at (0, 0), left loop on a circle around
    (-1, 0) and right loop around (1, 0), so the straight lines through the crossing
    are Ln-C-R0 and Rn-C-L0. With 3 per loop: L0 (-1, 1), L1 (-2, 0), L2 (-1, -1)."""
    g = Graph(clock)
    left = [f"L{i}" for i in range(nodes_per_loop)]
    right = [f"R{i}" for i in range(nodes_per_loop)]
    ring = ["C1"] + left + ["C2"] + right + ["C1"]
    for u, v in zip(ring, ring[1:]):
        g.add_edge(u, v, spacing)
    g.add_edge("C1", "C2", 0, turn=True)

    step = 2 * pi / (nodes_per_loop + 1)
    for i, node in enumerate(left):  # counterclockwise from the crossing
        a = (i + 1) * step
        g.pos[node] = (round(-1 + cos(a), 6), round(sin(a), 6))
    for i, node in enumerate(right):  # clockwise from the crossing
        a = pi - (i + 1) * step
        g.pos[node] = (round(1 + cos(a), 6), round(sin(a), 6))
    g.pos["C1"] = g.pos["C2"] = (0, 0)
    return g


def from_edges(edges, clock=time.monotonic):
    """Build a graph from a list of (u, v, weight), e.g. for the random track."""
    g = Graph(clock)
    for u, v, w in edges:
        g.add_edge(u, v, w)
    return g
