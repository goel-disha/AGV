"""Occupancy-grid environment for the AGV simulator."""

from dataclasses import dataclass
import numpy as np


@dataclass
class GridMap:
    width: int
    height: int
    resolution_m: float
    occupancy: np.ndarray

    @classmethod
    def empty(cls, width: int, height: int, resolution_m: float) -> "GridMap":
        return cls(width, height, resolution_m,
                   np.zeros((height, width), dtype=np.uint8))

    def add_rectangle(self, x0: int, y0: int, x1: int, y1: int) -> None:
        self.occupancy[max(0, y0):min(self.height, y1 + 1),
                       max(0, x0):min(self.width, x1 + 1)] = 1

    def is_free(self, x: int, y: int) -> bool:
        return (0 <= x < self.width and 0 <= y < self.height
                and self.occupancy[y, x] == 0)

    def world(self, cell):
        x, y = cell
        return np.array([x * self.resolution_m, y * self.resolution_m], dtype=float)
