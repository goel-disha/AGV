"""Run the M1 static-warehouse A* experiment."""

import time
from pathlib import Path

import yaml

from .grid import GridMap
from .astar import astar
from .metrics import path_length, collision_count
from .visualization import plot_result


def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main():
    root = Path(__file__).resolve().parents[2]
    cfg = load_config(root / "configs" / "static_warehouse.yaml")

    e = cfg["environment"]
    grid = GridMap.empty(e["width"], e["height"], e["resolution_m"])
    for obstacle in cfg["obstacles"]:
        grid.add_rectangle(*obstacle)

    start = tuple(cfg["start"])
    goal = tuple(cfg["goal"])

    t0 = time.perf_counter()
    path = astar(grid, start, goal)
    planning_ms = (time.perf_counter() - t0) * 1000

    print(f"Path cells: {len(path)}")
    print(f"Path length: {path_length(path, grid.resolution_m):.2f} m")
    print(f"Planning time: {planning_ms:.2f} ms")
    print(f"Collision cells: {collision_count(grid, path)}")

    out = root / "results"
    out.mkdir(exist_ok=True)
    fig = plot_result(grid, path, start, goal)
    fig.savefig(out / "m1_astar_static_warehouse.png", dpi=180)
    print(f"Saved: {out / 'm1_astar_static_warehouse.png'}")


if __name__ == "__main__":
    main()
