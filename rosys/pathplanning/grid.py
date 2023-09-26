from __future__ import annotations

from typing import overload

import numpy as np

from ..geometry import Point


class Grid:

    def __init__(self, size, bbox) -> None:
        self.size = size
        self.bbox = bbox

    @staticmethod
    def from_points(points: list[Point], pixel_size: float, num_layers: int, *, padding: float = 0) -> Grid:
        min_x = min_y = np.inf
        max_x = max_y = -np.inf
        for point in points:
            min_x = min(min_x, point.x - 2.0 * pixel_size - padding)
            min_y = min(min_y, point.y - 2.0 * pixel_size - padding)
            max_x = max(max_x, point.x + 2.0 * pixel_size + padding)
            max_y = max(max_y, point.y + 2.0 * pixel_size + padding)

        width = int((max_x - min_x) / pixel_size) + 1
        height = int((max_y - min_y) / pixel_size) + 1
        return Grid((height, width, num_layers), (min_x, min_y, max_x - min_x, max_y - min_y))

    @property
    def pixel_size(self) -> float:
        h_size = self.bbox[2] / self.size[1]
        v_size = self.bbox[3] / self.size[0]
        return (h_size + v_size) / 2.0

    def to_grid(self, x: float, y: float) -> tuple[float, float]:
        row = (y - self.bbox[1]) / self.bbox[3] * self.size[0] - 0.5
        col = (x - self.bbox[0]) / self.bbox[2] * self.size[1] - 0.5
        return row, col

    def to_3d_grid(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        row = (y - self.bbox[1]) / self.bbox[3] * self.size[0] - 0.5
        col = (x - self.bbox[0]) / self.bbox[2] * self.size[1] - 0.5
        layer = (yaw / 2.0 / np.pi * self.size[2]) % self.size[2]
        return row, col, layer

    @overload
    def from_grid(self, row: float, col: float) -> tuple[float, float]: ...

    @overload
    def from_grid(self, row: np.ndarray, col: np.ndarray) -> tuple[np.ndarray, np.ndarray]: ...

    def from_grid(self, row: float | np.ndarray, col: float | np.ndarray) -> tuple[float | np.ndarray, float | np.ndarray]:
        x = (col + 0.5) / self.size[1] * self.bbox[2] + self.bbox[0]
        y = (row + 0.5) / self.size[0] * self.bbox[3] + self.bbox[1]
        return x, y

    def from_3d_grid(self, row, col, layer) -> tuple[float, float, float]:
        x = (col + 0.5) / self.size[1] * self.bbox[2] + self.bbox[0]
        y = (row + 0.5) / self.size[0] * self.bbox[3] + self.bbox[1]
        yaw = layer / self.size[2] * 2.0 * np.pi
        return x, y, yaw

    def contains(self, point: Point, *, padding: float = 0) -> bool:
        return self.bbox[0] + padding < point.x < self.bbox[0] + self.bbox[2] - padding \
            and self.bbox[1] + padding < point.y < self.bbox[1] + self.bbox[3] - padding
