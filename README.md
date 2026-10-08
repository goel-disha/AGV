# Autonomous Ground Vehicle — Static Obstacle Avoidance

A research-oriented AGV simulation for **static obstacle avoidance, global planning, path tracking, and sensor-based navigation**.

## Current milestone: M1 — Global A* Planning

The first milestone provides:
- 2D occupancy-grid warehouse environment
- rectangular static obstacles
- 8-connected A* global planner
- collision-free path generation
- path length and planning-time metrics
- matplotlib visualization
- automated tests

## Roadmap

- [x] M1 — A* global planning in static occupancy grid
- [ ] M2 — Differential-drive AGV kinematics and path tracking
- [ ] M3 — Pure Pursuit / Stanley-style tracking comparison
- [ ] M4 — Simulated LiDAR sensor
- [ ] M5 — Local static-obstacle avoidance
- [ ] M6 — Hybrid global + local planning
- [ ] M7 — Quantitative evaluation and ablation study
- [ ] M8 — ROS 2 + Gazebo integration

## Quick start

```bash
python -m pip install -r requirements.txt
python -m src.agv_sim.simulate
pytest -q
```

## Project structure

```text
configs/          Simulation configuration
src/agv_sim/      Planning, simulation and visualization code
tests/            Automated tests
docs/             Research notes and roadmap
results/          Generated experiment figures
```

## Research objective

The long-term objective is to evaluate how a mobile AGV can navigate a structured industrial environment containing static obstacles while maintaining safe, efficient and smooth motion.

Metrics will include path length, planning time, tracking error, minimum obstacle clearance, success rate and computation time.

## Author

Disha Goel — B.Tech EEE, BIT Mesra
