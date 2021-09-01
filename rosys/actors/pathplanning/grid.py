import numpy as np


class Grid:

    def __init__(self, size, bbox):
        self.size = size
        self.bbox = bbox

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
