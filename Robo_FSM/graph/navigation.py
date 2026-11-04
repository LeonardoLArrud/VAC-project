from math import inf

from graph.path import dijkstra

DYNAMIC_BLOCK_TIME = 5.0  # seconds an edge stays blocked after a detection


class Navigator:
    """Drives the car on the graph.

    With a goal: follows the shortest path and replans around obstacles.
    Without a goal (or once it is reached, or while it is unreachable): keeps going,
    straight when possible, and only goes back if there is no other way. If the
    sensor sees an obstacle meanwhile, the car turns around and then visits every
    node of the track (to_visit), before going back to just driving.

    Static obstacles: block their edges on the graph before creating the Navigator.
    Dynamic obstacles: call obstacle_ahead() when the sensor sees one on the edge the
    car is driving on; the car turns around and drives back to the node it came from.
    The edge is blocked for DYNAMIC_BLOCK_TIME seconds, then can be used again.
    Sharp turns are only allowed while some edge of the track is blocked.

    The controller calls advance() every time the car reaches a node, also when it
    comes back to a node after turning around.

    facing: neighbor of start the car points to when placed. With it, the controller
    gets the first move before driving:
        first_maneuver(nav.graph, nav.current, nav.facing, nav.next_node())
    """

    def __init__(self, graph, start, goal=None, facing=None):
        self.graph = graph
        self.start = start
        self.current = start
        self.previous = None
        self.facing = facing  # neighbor of start the car points to when placed (or None)
        self.returning_from = None  # node of the blocked edge while driving back
        self.history = [start]  # every node the car went through, in order
        self.last_visit = {start: 0}  # node -> index in history of its last visit
        self.set_goal(goal)

    @property
    def allow_turns(self):
        return self.graph.has_blocked()

    def set_goal(self, goal):
        """New delivery, or None to just keep driving."""
        self.goal = goal
        self.to_visit = set()  # visit-all mode (no goal, after an obstacle)
        self.replan()

    def replan(self):
        target = self.goal if self.goal is not None else self._visit_target()
        if target is None:
            self.path, self.distance = [], inf
        else:
            self.path, self.distance = dijkstra(
                self.graph, self.current, target, self.allow_turns
            )
        if self.returning_from is not None:
            return  # still driving back: the rest is decided at the node

        if len(self.path) > 1 and self.graph.is_turn(self.path[0], self.path[1]):
            self._cross_to(self.path[1])
            self.path = self.path[1:]
        elif len(self.path) <= 1 and self._cruise_next() == self.previous:
            self._sharp_turn_instead_of_u_turn()

    def _sharp_turn_instead_of_u_turn(self):
        """Cruising with the way ahead blocked: at the 8's crossing, turn sharply
        (switch C1 <-> C2 on the spot) if that leads somewhere, instead of going back."""
        for twin, _ in self.graph.neighbors(self.current, allow_turns=True):
            if self.graph.is_turn(self.current, twin) and any(self.graph.neighbors(twin)):
                self._cross_to(twin)
                return

    def _cross_to(self, twin):
        self.current = twin
        self._visit(twin)

    def _visit(self, node):
        self.last_visit[node] = len(self.history)
        self.history.append(node)
        self.to_visit.discard(node)

    def _visit_target(self):
        """Visit-all mode: the next node still to visit. A neighbor if there is one
        (straight on first), else the closest one. None when nothing is left."""
        if not self.to_visit:
            return None
        free = [v for v, _ in self.graph.neighbors(self.current)]
        new = [v for v in free if v in self.to_visit]
        straight = self.graph.straight_ahead(self.previous, self.current)
        if straight in new:
            return straight
        if new:
            return new[0]
        return min(
            self.to_visit,
            key=lambda n: dijkstra(self.graph, self.current, n, self.allow_turns)[1],
        )

    def next_node(self):
        """Node the car is heading to, or None if every edge around it is blocked."""
        if self.returning_from is not None:
            return self.current  # driving back to the node it came from
        if len(self.path) > 1:
            return self.path[1]
        return self._cruise_next()

    def _cruise_next(self):
        """No route to follow: go straight, else any way but back (the one visited
        longest ago, so the car never circles one part of the track), else back."""
        free = [v for v, _ in self.graph.neighbors(self.current)]
        if self.previous is None and self.facing in free:
            return self.facing  # just placed: drive ahead the way the car points
        straight = self.graph.straight_ahead(self.previous, self.current)
        if straight in free:
            return straight
        ahead = [v for v in free if v != self.previous]
        if ahead:
            return min(ahead, key=lambda v: self.last_visit.get(v, -1))
        return free[0] if free else None

    def arrived(self):
        return self.current == self.goal

    def obstacle_ahead(self, duration=DYNAMIC_BLOCK_TIME):
        """The sensor sees an obstacle on the edge to next_node(): block it for a
        while; the car turns around and drives back to the node it came from.
        Without a goal, it then visits every node of the track (visit-all mode)."""
        blocked = self.next_node()
        self.graph.block_edge(self.current, blocked, duration)
        self.returning_from = blocked
        if self.goal is None:
            self.to_visit = set(self.graph.adj) - {self.current}
        self.replan()

    def advance(self):
        """The car reached the next node (e.g. intersection detected).
        Replans so that edges freed in the meantime can be used again."""
        if self.returning_from is not None:  # back at the node, from the blocked side
            self.previous, self.returning_from = self.returning_from, None
            self._visit(self.current)
            self.replan()
            return
        nxt = self.next_node()
        if nxt is None:
            raise RuntimeError("every edge around the car is blocked")
        self.previous = self.current
        self.current = nxt
        self._visit(nxt)
        self.replan()
