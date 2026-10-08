"""Differential-drive AGV kinematic model."""

from dataclasses import dataclass
from math import cos, sin


@dataclass
class AGVState:
    x: float
    y: float
    theta: float


@dataclass
class DifferentialDriveAGV:
    wheel_radius_m: float = 0.05
    wheel_base_m: float = 0.30
    max_linear_mps: float = 1.0
    max_angular_rps: float = 2.5
    state: AGVState = None

    def __post_init__(self):
        if self.state is None:
            self.state = AGVState(0.0, 0.0, 0.0)

    def reset(self, x, y, theta=0.0):
        self.state = AGVState(float(x), float(y), float(theta))

    def step(self, linear_mps, angular_rps, dt):
        """Integrate unicycle-equivalent differential-drive kinematics."""
        v = max(-self.max_linear_mps, min(self.max_linear_mps, linear_mps))
        omega = max(-self.max_angular_rps, min(self.max_angular_rps, angular_rps))

        self.state.x += v * cos(self.state.theta) * dt
        self.state.y += v * sin(self.state.theta) * dt
        self.state.theta = _wrap_angle(self.state.theta + omega * dt)
        return self.state


def _wrap_angle(angle):
    while angle > 3.141592653589793:
        angle -= 2.0 * 3.141592653589793
    while angle <= -3.141592653589793:
        angle += 2.0 * 3.141592653589793
    return angle
