# Research Roadmap

## M1 — Global planning
A* on a 2D occupancy grid with static rectangular obstacles.

## M2 — Vehicle model
Implement a differential-drive AGV state:
- x, y
- heading
- linear velocity
- angular velocity

Add wheel constraints and discrete-time integration.

## M3 — Path tracking
Compare:
- waypoint pursuit
- Pure Pursuit
- Stanley-style steering

Measure lateral error, heading error and completion time.

## M4 — LiDAR
Implement a 2D ray-casting LiDAR against the occupancy grid. Generate scans and visualize sensor returns.

## M5 — Local avoidance
Implement a local planner suitable for static obstacles and compare it with the global path.

## M6 — Hybrid planner
Combine A* global planning with local obstacle avoidance and safety constraints.

## M7 — Evaluation
Run repeated experiments over different obstacle densities and start/goal pairs.

Primary metrics:
- success rate
- path length
- planning time
- minimum clearance
- tracking error
- control effort
- computation time
