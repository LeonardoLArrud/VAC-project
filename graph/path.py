import heapq
from itertools import count
from math import inf


def dijkstra(graph, start, goal, allow_turns=False):
    """Shortest path avoiding blocked edges (and sharp turns unless allow_turns).
    Returns (path, distance), or ([], inf) if the goal is unreachable."""
    if start not in graph.adj or goal not in graph.adj:
        raise KeyError("start or goal not in graph")

    dist = {start: 0}
    prev = {}
    tie = count()  # avoids comparing nodes of different types in the heap
    heap = [(0, next(tie), start)]

    while heap:
        d, _, u = heapq.heappop(heap)
        if u == goal:
            return _rebuild_path(prev, start, goal), d
        if d > dist[u]:
            continue  # outdated heap entry
        for v, w in graph.neighbors(u, allow_turns):
            if d + w < dist.get(v, inf):
                dist[v] = d + w
                prev[v] = u
                heapq.heappush(heap, (d + w, next(tie), v))

    return [], inf


def _rebuild_path(prev, start, goal):
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    return path[::-1]
