#!/usr/bin/env python3
import numpy as np
from .binary_renderer import BinaryRenderer


class RobotRenderer:

    def __init__(self, outline):
        self.outline = outline

    @staticmethod
    def from_size(width, length, x_shift=0):
        return RobotRenderer([
            [-length / 2 + x_shift, -width / 2],
            [+length / 2 + x_shift, -width / 2],
            [+length / 2 + x_shift, +width / 2],
            [-length / 2 + x_shift, +width / 2],
        ])

    def render(self, pixel_size, yaw=0):
        radius = np.linalg.norm(self.outline, axis=1).max()
        width = 2 * int(np.ceil(radius / pixel_size)) + 1

        R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        self.rendered_outline = np.array(self.outline).dot(R.T) / pixel_size + width // 2

        renderer = BinaryRenderer((width, width))
        renderer.map.fill(False)
        renderer.polygon(self.rendered_outline)
        return renderer.map
