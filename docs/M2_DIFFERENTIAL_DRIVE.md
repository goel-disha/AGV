# M2 — Differential-Drive AGV and Path Tracking

## Objective

Move from a point-robot A* planner to a kinematic AGV that physically follows the planned path.

## Vehicle model

The AGV uses differential-drive/unicycle-equivalent kinematics:

x_dot = v cos(theta)

y_dot = v sin(theta)

theta_dot = omega

where:
- v is linear velocity
- omega is angular velocity
- theta is vehicle heading

Velocity limits are applied before integration.

## Controller

M2 uses Pure Pursuit. A lookahead point is selected on the reference path and curvature is computed from the heading error.

This is intentionally a simple baseline. Later experiments can compare Pure Pursuit against other tracking controllers.

## Metrics

The simulation reports:
- trajectory length
- tracking RMSE
- maximum tracking error
- final goal error
- success/failure

## Why this matters

M1 only proves that a collision-free geometric path exists. M2 tests whether an actual nonholonomic AGV model can follow that path.

The next major research step is adding a simulated LiDAR and using it to reason about obstacle clearance and local avoidance.
