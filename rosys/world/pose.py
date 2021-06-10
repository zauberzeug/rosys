from __future__ import annotations
from pydantic import BaseModel
import numpy as np


class Pose(BaseModel):

    x: float = 0
    y: float = 0
    yaw: float = 0

    def __str__(self):
        return '%.3f, %.3f, %.1f deg' % (self.x, self.y, np.rad2deg(self.yaw))

    def distance(self, other: Pose) -> float:

        return np.sqrt((other.x - self.x)**2 + (other.y - self.y)**2)

    def projected_distance(self, other: Pose) -> float:
        """Returnes the projected distance from `self` to `other` in direction of `other`."""

        def d(p): return np.sqrt(p.x**2 + p.y**2) * np.cos(other.yaw - np.arctan2(p.y, p.x))
        return d(other) - d(self)
