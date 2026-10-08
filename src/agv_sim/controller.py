"""Path tracking controllers."""

from math import atan2, cos, hypot, sin


def wrap_angle(angle):
    while angle > 3.141592653589793:
        angle -= 2.0 * 3.141592653589793
    while angle <= -3.141592653589793:
        angle += 2.0 * 3.141592653589793
    return angle


class PurePursuitController:
    """Geometric controller for tracking a sequence of XY waypoints."""

    def __init__(self, lookahead_m=0.8, linear_speed_mps=0.55,
                 max_angular_rps=2.5, goal_tolerance_m=0.25):
        self.lookahead_m = lookahead_m
        self.linear_speed_mps = linear_speed_mps
        self.max_angular_rps = max_angular_rps
        self.goal_tolerance_m = goal_tolerance_m

    def command(self, state, path):
        if not path:
            return 0.0, 0.0, True

        goal = path[-1]
        goal_dist = hypot(goal[0] - state.x, goal[1] - state.y)
        if goal_dist <= self.goal_tolerance_m:
            return 0.0, 0.0, True

        # Choose the first waypoint at least lookahead distance away.
        target = path[-1]
        for point in path:
            if hypot(point[0] - state.x, point[1] - state.y) >= self.lookahead_m:
                target = point
                break

        dx = target[0] - state.x
        dy = target[1] - state.y
        alpha = wrap_angle(atan2(dy, dx) - state.theta)

        curvature = 2.0 * sin(alpha) / max(self.lookahead_m, 1e-6)
        omega = curvature * self.linear_speed_mps
        omega = max(-self.max_angular_rps, min(self.max_angular_rps, omega))
        return self.linear_speed_mps, omega, False
