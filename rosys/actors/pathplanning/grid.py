import numpy as np
from rosys.world.point import Point


class Grid:

    def __init__(self, size, bbox):
        self.size = size
        self.bbox = bbox

    @staticmethod
    def from_points(points: list[Point], pixel_size: float, num_layers: int):
        min_x = min_y = np.inf
        max_x = max_y = -np.inf
        for point in points:
            min_x = min(min_x, point.x - 2.0 * pixel_size)
            min_y = min(min_y, point.y - 2.0 * pixel_size)
            max_x = max(max_x, point.x + 2.0 * pixel_size)
            max_y = max(max_y, point.y + 2.0 * pixel_size)

        width = int((max_x - min_x) / pixel_size) + 1
        height = int((max_y - min_y) / pixel_size) + 1
        return Grid((height, width, num_layers), (min_x, min_y, max_x - min_x, max_y - min_y))

    @property
    def pixel_size(self):
        h_size = self.bbox[2] / self.size[1]
        v_size = self.bbox[3] / self.size[0]
        return (h_size + v_size) / 2.0

    def to_grid(self, x, y, yaw=None):
        row = (y - self.bbox[1]) / self.bbox[3] * self.size[0] - 0.5
        col = (x - self.bbox[0]) / self.bbox[2] * self.size[1] - 0.5
        if yaw is None:
            return row, col

        layer = (yaw / 2.0 / np.pi * self.size[2]) % self.size[2]
        return row, col, layer

    def from_grid(self, row, col, layer=None):
        x = (col + 0.5) / self.size[1] * self.bbox[2] + self.bbox[0]
        y = (row + 0.5) / self.size[0] * self.bbox[3] + self.bbox[1]
        if layer is None:
            return x, y

        yaw = layer / self.size[2] * 2.0 * np.pi
        return x, y, yaw
