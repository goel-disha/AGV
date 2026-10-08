"""M2 experiment: A* path followed by a differential-drive AGV."""

from math import atan2
from pathlib import Path

import matplotlib.pyplot as plt
import yaml

from .astar import astar
from .controller import PurePursuitController
from .grid import GridMap
from .tracking import trajectory_metrics
from .vehicle import DifferentialDriveAGV


def main():
    root = Path(__file__).resolve().parents[2]
    with open(root / "configs" / "static_warehouse.yaml", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    e = cfg["environment"]
    grid = GridMap.empty(e["width"], e["height"], e["resolution_m"])
    for obstacle in cfg["obstacles"]:
        grid.add_rectangle(*obstacle)

    start_cell = tuple(cfg["start"])
    goal_cell = tuple(cfg["goal"])
    path_cells = astar(grid, start_cell, goal_cell)
    path = [grid.world(p) for p in path_cells]
    goal = grid.world(goal_cell)
    start = grid.world(start_cell)

    # Start aligned with the first path segment.
    if len(path) > 1:
        theta0 = atan2(path[1][1] - path[0][1],
                       path[1][0] - path[0][0])
    else:
        theta0 = 0.0

    agv = DifferentialDriveAGV()
    agv.reset(start[0], start[1], theta0)
    controller = PurePursuitController(
        lookahead_m=0.8, linear_speed_mps=0.55, goal_tolerance_m=0.28
    )

    dt = 0.05
    max_time = 120.0
    trajectory = [(agv.state.x, agv.state.y, agv.state.theta)]
    t = 0.0

    while t < max_time:
        v, omega, done = controller.command(agv.state, path)
        if done:
            break
        agv.step(v, omega, dt)
        trajectory.append((agv.state.x, agv.state.y, agv.state.theta))
        t += dt

    metrics = trajectory_metrics(
        [(s[0], s[1]) for s in trajectory], path, goal,
        goal_tolerance=controller.goal_tolerance_m
    )

    print("=== AGV M2 — Differential Drive + Pure Pursuit ===")
    print(f"Simulation time: {t:.2f} s")
    print(f"Trajectory length: {metrics['trajectory_length_m']:.2f} m")
    print(f"Tracking RMSE: {metrics['tracking_rmse_m']:.3f} m")
    print(f"Maximum tracking error: {metrics['max_tracking_error_m']:.3f} m")
    print(f"Final goal error: {metrics['goal_error_m']:.3f} m")
    print(f"Success: {metrics['success']}")

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.imshow(grid.occupancy, origin="lower", cmap="Greys",
              extent=[0, e["width"], 0, e["height"]])
    ax.plot([p[0] / e["resolution_m"] for p in path],
            [p[1] / e["resolution_m"] for p in path],
            "--", linewidth=1.5, label="A* reference path")
    ax.plot([s[0] / e["resolution_m"] for s in trajectory],
            [s[1] / e["resolution_m"] for s in trajectory],
            linewidth=2, label="AGV trajectory")
    ax.scatter([start_cell[0]], [start_cell[1]], s=70, label="Start")
    ax.scatter([goal_cell[0]], [goal_cell[1]], s=70, label="Goal")
    ax.set_xlabel("X [grid cell]")
    ax.set_ylabel("Y [grid cell]")
    ax.set_title("Differential-Drive AGV Tracking an A* Path")
    ax.legend()
    ax.grid(alpha=0.15)
    fig.tight_layout()
    out = root / "results"
    out.mkdir(exist_ok=True)
    fig.savefig(out / "m2_differential_drive_tracking.png", dpi=180)
    print(f"Saved: {out / 'm2_differential_drive_tracking.png'}")


if __name__ == "__main__":
    main()
