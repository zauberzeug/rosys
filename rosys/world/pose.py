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

    @property
    def matrix(self):
        return np.array([
            [np.cos(self.yaw), -np.sin(self.yaw), self.x],
            [np.sin(self.yaw), np.cos(self.yaw), self.y],
            [0, 0, 1],
        ])

    @staticmethod
    def from_matrix(M, time: float = 0):
        return Pose(x=M[0, -1], y=M[1, -1], yaw=np.arctan2(M[1, 0], M[0, 0]), time=time)

    def __str__(self):
        return '%.3f, %.3f, %.1f deg' % (self.x, self.y, np.rad2deg(self.yaw))

    def distance(self, other: Pose) -> float:
        return self.point.distance(other.point)

    def projected_distance(self, other: Pose) -> float:
        return self.point.projected_distance(other.point, other.yaw)

    def __iadd__(self, step: PoseStep):
        self.x += step.linear * np.cos(self.yaw)
        self.y += step.linear * np.sin(self.yaw)
        self.yaw += step.angular
        self.time = step.time
        return self

    def transform(self, point: Point) -> Point:
        return Point(
            x=self.x + point.x * np.cos(self.yaw) - point.y * np.sin(self.yaw),
            y=self.y + point.x * np.sin(self.yaw) + point.y * np.cos(self.yaw),
        )

    def rotate(self, angle: float) -> Pose:
        return Pose(x=self.x, y=self.y, yaw=self.yaw+angle, time=self.time)
