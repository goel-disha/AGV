import numpy as np

from src.agv_sim.grid import GridMap
from src.agv_sim.astar import astar
from src.agv_sim.metrics import collision_count


def test_astar_finds_collision_free_path():
    grid = GridMap.empty(40, 30, 0.2)
    grid.add_rectangle(10, 5, 20, 18)
    path = astar(grid, (2, 2), (35, 25))

    assert path
    assert path[0] == (2, 2)
    assert path[-1] == (35, 25)
    assert collision_count(grid, path) == 0


def test_blocked_goal_returns_no_path():
    grid = GridMap.empty(20, 20, 0.2)
    grid.add_rectangle(0, 0, 19, 19)
    assert astar(grid, (1, 1), (10, 10)) == []
