# Autonomous Ground Vehicle — Static Obstacle Avoidance

A research-oriented AGV simulation for **static obstacle avoidance, global planning, path tracking, and sensor-based navigation**.

## Current status

- [x] **M1 — A* global planning**
- [x] **M2 — Differential-drive AGV + Pure Pursuit tracking**
- [ ] M3 — Tracking-controller comparison
- [ ] M4 — Simulated 2D LiDAR
- [ ] M5 — Local static-obstacle avoidance
- [ ] M6 — Hybrid global + local planner
- [ ] M7 — Quantitative evaluation / ablation study
- [ ] M8 — ROS 2 + Gazebo integration

## M1 — Global planning

A known warehouse is represented as a binary occupancy grid. An 8-connected A* planner generates a collision-free reference path.

## M2 — Actual AGV motion

The reference path is converted into metric coordinates and followed by a differential-drive AGV model:

```text
x_dot     = v cos(theta)
y_dot     = v sin(theta)
theta_dot = omega
```

A Pure Pursuit controller generates angular velocity from a lookahead point.

M2 reports:
- trajectory length
- tracking RMSE
- maximum tracking error
- final goal error
- navigation success

## Quick start

```bash
python -m pip install -r requirements.txt

# M1
python -m src.agv_sim.simulate

# M2
python -m src.agv_sim.simulate_m2

# Tests
pytest -q
```

## Repository structure

```text
AGV/
├── configs/
│   └── static_warehouse.yaml
├── src/agv_sim/
│   ├── grid.py
│   ├── astar.py
│   ├── metrics.py
│   ├── visualization.py
│   ├── vehicle.py
│   ├── controller.py
│   ├── tracking.py
│   ├── simulate.py
│   └── simulate_m2.py
├── tests/
├── docs/
│   ├── ROADMAP.md
│   ├── RESEARCH_NOTES.md
│   └── M2_DIFFERENTIAL_DRIVE.md
└── results/
```

## Research objective

The long-term objective is to evaluate how an autonomous AGV can navigate a structured industrial environment containing static obstacles while maintaining safe, efficient and smooth motion.

The evaluation will progressively include:

- path length
- planning time
- tracking error
- minimum obstacle clearance
- collision rate
- navigation success rate
- control effort
- computation time

## Research direction

The project deliberately separates **global planning**, **vehicle dynamics**, **perception**, and **local avoidance**. This makes it possible to compare algorithms under the same map and start/goal conditions.

The next major milestone is a simulated **2D LiDAR** that can observe static obstacles from the moving AGV. This will allow the project to move from purely map-based planning toward sensor-aware obstacle avoidance.

## Author

Disha Goel — B.Tech EEE, BIT Mesra
