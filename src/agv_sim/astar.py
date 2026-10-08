"""8-connected A* planner."""

from heapq import heappush, heappop
from math import hypot


NEIGHBORS = [
    (-1, -1, 2 ** 0.5), (0, -1, 1.0), (1, -1, 2 ** 0.5),
    (-1,  0, 1.0),                       (1,  0, 1.0),
    (-1,  1, 2 ** 0.5), (0,  1, 1.0), (1,  1, 2 ** 0.5),
]


def heuristic(a, b):
    return hypot(b[0] - a[0], b[1] - a[1])


def astar(grid, start, goal):
    if not grid.is_free(*start) or not grid.is_free(*goal):
        return []

    frontier = []
    heappush(frontier, (0.0, start))
    came_from = {start: None}
    cost = {start: 0.0}

    while frontier:
        _, current = heappop(frontier)

        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for dx, dy, step_cost in NEIGHBORS:
            nxt = (current[0] + dx, current[1] + dy)
            if not grid.is_free(*nxt):
                continue

            new_cost = cost[current] + step_cost
            if nxt not in cost or new_cost < cost[nxt]:
                cost[nxt] = new_cost
                priority = new_cost + heuristic(nxt, goal)
                heappush(frontier, (priority, nxt))
                came_from[nxt] = current

    return []
