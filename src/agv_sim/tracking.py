"""Trajectory tracking and evaluation utilities."""

from math import hypot


def nearest_path_distance(x, y, path):
    if not path:
        return float("inf")
    return min(hypot(x - px, y - py) for px, py in path)


def trajectory_metrics(trajectory, path, goal, goal_tolerance=0.25):
    if not trajectory:
        return {
            "tracking_rmse_m": float("inf"),
            "max_tracking_error_m": float("inf"),
            "trajectory_length_m": 0.0,
            "goal_error_m": float("inf"),
            "success": False,
        }

    errors = [nearest_path_distance(s[0], s[1], path) for s in trajectory]
    length = sum(
        hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(trajectory[:-1], trajectory[1:])
    )
    goal_error = hypot(trajectory[-1][0] - goal[0],
                        trajectory[-1][1] - goal[1])

    return {
        "tracking_rmse_m": (sum(e * e for e in errors) / len(errors)) ** 0.5,
        "max_tracking_error_m": max(errors),
        "trajectory_length_m": length,
        "goal_error_m": goal_error,
        "success": goal_error <= goal_tolerance,
    }
