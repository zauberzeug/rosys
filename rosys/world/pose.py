from __future__ import annotations  # NOTE: for PEP 563 (postponed evaluation of annotations)
from pydantic import BaseModel
import numpy as np
from .point import Point


class PoseStep(BaseModel):

    linear: float
    angular: float
    time: float


class Pose(BaseModel):

    x: float = 0
    y: float = 0
    yaw: float = 0
    time: float = 0

    @property
    def point(self) -> Point:
        return Point(x=self.x, y=self.y)

    def __str__(self):
        return '%.3f, %.3f, %.1f deg' % (self.x, self.y, np.rad2deg(self.yaw))

    def distance(self, other: Pose) -> float:

        return self.point.distance(other.point)

    def projected_distance(self, other: Pose) -> float:
        """Returnes the projected distance from `self` to `other` in direction of `other`."""

        def d(p): return np.sqrt(p.x**2 + p.y**2) * np.cos(other.yaw - np.arctan2(p.y, p.x))
        return d(other) - d(self)

    def __iadd__(self, step: PoseStep):

        self.x += step.linear * np.cos(self.yaw)
        self.y += step.linear * np.sin(self.yaw)
        self.yaw += step.angular
        self.time = step.time
        return self
