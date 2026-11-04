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
    (px, py), (cx, cy), (nx, ny) = _positions(graph, (prev, cur, nxt), cur)
    return _turn((cx - px, cy - py), (nx - cx, ny - cy))


def first_maneuver(graph, cur, facing, nxt):
    """Move at the start: the car stands at cur pointing at its neighbor facing,
    and must leave towards nxt (first node of the path)."""
    if nxt == facing:
        return STRAIGHT
    (fx, fy), (cx, cy), (nx, ny) = _positions(graph, (facing, cur, nxt), cur)
    return _turn((fx - cx, fy - cy), (nx - cx, ny - cy))


def _positions(graph, nodes, cur):
    try:
        return [graph.pos[n] for n in nodes]
    except KeyError:
        raise ValueError(f"node positions are needed to choose a turn at {cur}")


def _turn(a, b):
    """a: direction the car points to, b: direction it must leave in."""
    (ax, ay), (bx, by) = a, b
    angle = degrees(atan2(ax * by - ay * bx, ax * bx + ay * by))
    if abs(angle) < STRAIGHT_TOLERANCE:
        return STRAIGHT
    if abs(angle) > 180 - STRAIGHT_TOLERANCE:
        return U_TURN
    return LEFT if angle > 0 else RIGHT


def maneuvers(graph, path, previous=None):
    """Maneuver at each node of path except the last one (the goal).
    previous: node the car came from before path[0], if known."""
    # a sharp-turn edge (C1 -> C2) is crossed on the spot: not a separate node
    path = [u for u, v in zip(path, path[1:]) if not graph.is_turn(u, v)] + path[-1:]
    prevs = [previous] + path[:-1]
    return [maneuver(graph, p, c, n) for p, c, n in zip(prevs, path, path[1:])]
