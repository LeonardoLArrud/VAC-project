"""A fake car on a track, to test the graph logic without the real robot.

Static obstacles are known in advance (blocked in the graph right away).
Dynamic obstacles exist only in the "world": the car learns about one when its
sensor sees it, i.e. when the edge it is about to take has one.
"""

from math import hypot

from graph.maneuver import maneuver
from graph.navigation import Navigator

OBSTACLE_AT = 0.5  # dynamic obstacles sit in the middle of their edge
CAR_STOP = 0.25  # where the car stops on the edge when the sensor sees one


class Simulation:
    def __init__(self, graph, start):
        self.graph = graph
        self.nav = Navigator(graph, start)
        self.static = set()  # edges blocked forever, known by the car
        self.hidden = set()  # edges with a dynamic obstacle, unknown until seen
        self.log = []

    def toggle_static(self, u, v):
        e = frozenset((u, v))
        if e in self.static:
            self.static.remove(e)
            self.graph.unblock_edge(u, v)
        else:
            self.static.add(e)
            self.graph.block_edge(u, v)
        self.nav.replan()

    def toggle_dynamic(self, u, v):
        self.hidden ^= {frozenset((u, v))}

    def set_goal(self, node):
        self.nav.set_goal(node)
        self._log(f"new goal: {node}")

    def move_car(self, node):
        self.nav = Navigator(self.graph, node, self.nav.goal)  # node = its new start
        self._log(f"car placed at {node} (new start)")

    def step(self):
        """One move of the car: to the next node or, if the edge has a dynamic
        obstacle, up to it (sensor sees it, car turns around) and back next step.
        Returns what happened, as a log line."""
        nav = self.nav

        if nav.arrived() and (nav.goal == nav.start):
            return self._log(f"waiting at start {nav.start} for the next delivery")

        here = nav.current

        if nav.returning_from is not None:
            nav.advance()
            text = f"back at {here} after turning around"
        else:
            nxt = nav.next_node()
            if nxt is None:
                return self._log("stuck: every edge around the car is blocked")
            if frozenset((here, nxt)) in self.hidden:
                nav.obstacle_ahead()
                return self._log(f"obstacle seen on {here}-{nxt}: turning around")
            move = maneuver(self.graph, nav.previous, here, nxt)
            nav.advance()
            text = f"{move} at {here} -> {nav.current}"

        if nav.arrived() and (nav.current != nav.start):
            nav.set_goal(nav.start)  # delivery done: go back to start
            text += "   DELIVERED, going back to start"

        return self._log(text)

    def car_position(self):
        """(x, y) of the car: on its node, or in front of the obstacle while driving back."""
        here = self.graph.pos[self.nav.current]
        if self.nav.returning_from is None:
            return here
        there = self.graph.pos[self.nav.returning_from]
        return tuple(h + CAR_STOP * (t - h) for h, t in zip(here, there))

    def _log(self, text):
        self.log.append(text)
        return text


def edges(graph):
    """Each drawable edge once, as (u, v). Sharp-turn edges (same spot) are left out."""
    seen = set()
    for u in graph.adj:
        for v in graph.adj[u]:
            e = frozenset((u, v))
            if e not in seen and not graph.is_turn(u, v):
                seen.add(e)
                yield u, v


def nearest_node(graph, x, y):
    """(node, distance) of the node closest to the point (x, y)."""
    return min(
        ((n, hypot(x - px, y - py)) for n, (px, py) in graph.pos.items()),
        key=lambda item: item[1],
    )


def nearest_edge(graph, x, y):
    """((u, v), distance) of the edge closest to the point (x, y)."""
    return min(
        (((u, v), _segment_distance((x, y), graph.pos[u], graph.pos[v]))
         for u, v in edges(graph)),
        key=lambda item: item[1],
    )


def _segment_distance(p, a, b):
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))  # closest point must stay on the segment
    return hypot(px - (ax + t * dx), py - (ay + t * dy))
