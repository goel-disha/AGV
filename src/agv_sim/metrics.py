"""Evaluation metrics for planned paths."""

from math import hypot


def path_length(path, resolution_m=1.0):
    if len(path) < 2:
        return 0.0
    return sum(
        hypot(b[0] - a[0], b[1] - a[1]) * resolution_m
        for a, b in zip(path[:-1], path[1:])
    )


def collision_count(grid, path):
    return sum(not grid.is_free(*p) for p in path)
