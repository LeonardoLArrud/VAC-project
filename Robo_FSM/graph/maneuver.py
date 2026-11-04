"""Translate a path on the graph into what the car must do at each node."""

from math import atan2, degrees

FOLLOW = "follow"  # no choice at this node: keep following the line
STRAIGHT = "straight"
LEFT = "left"
RIGHT = "right"
U_TURN = "u_turn"

STRAIGHT_TOLERANCE = 30  # degrees: a smaller deviation still counts as straight


def maneuver(graph, prev, cur, nxt):
    """What the car must do at cur to go prev -> cur -> nxt."""
    if nxt == prev:
        return U_TURN
    if prev is None or len(graph.adj[cur]) <= 2:
        return FOLLOW
    try:
        (px, py), (cx, cy), (nx, ny) = graph.pos[prev], graph.pos[cur], graph.pos[nxt]
    except KeyError:
        raise ValueError(f"node positions are needed to choose a turn at {cur}")
    ax, ay = cx - px, cy - py  # direction the car arrives from
    bx, by = nx - cx, ny - cy  # direction it must leave in
    angle = degrees(atan2(ax * by - ay * bx, ax * bx + ay * by))
    if abs(angle) < STRAIGHT_TOLERANCE:
        return STRAIGHT
    return LEFT if angle > 0 else RIGHT


def maneuvers(graph, path, previous=None):
    """Maneuver at each node of path except the last one (the goal).
    previous: node the car came from before path[0], if known."""
    # a sharp-turn edge (C1 -> C2) is crossed on the spot: not a separate node
    path = [u for u, v in zip(path, path[1:]) if not graph.is_turn(u, v)] + path[-1:]
    prevs = [previous] + path[:-1]
    return [maneuver(graph, p, c, n) for p, c, n in zip(prevs, path, path[1:])]
