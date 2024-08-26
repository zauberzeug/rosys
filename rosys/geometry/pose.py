from __future__ import annotations

from dataclasses import dataclass
from typing import overload

import numpy as np

from .. import helpers
from .line import Line
from .point import Point
from .point3d import Point3d


@dataclass(slots=True, kw_only=True)
class PoseStep:
    linear: float
    angular: float
    time: float


@dataclass(slots=True, kw_only=True)
class Pose:
    x: float = 0
    y: float = 0
    yaw: float = 0
    time: float = 0

    @property
    def point(self) -> Point:
        return Point(x=self.x, y=self.y)

    def point_3d(self, z: float = 0) -> Point3d:
        return Point3d(x=self.x, y=self.y, z=z)

    @property
    def matrix(self) -> np.ndarray:
        return np.array([
            [np.cos(self.yaw), -np.sin(self.yaw), self.x],
            [np.sin(self.yaw), np.cos(self.yaw), self.y],
            [0, 0, 1],
        ])

    @property
    def inv_matrix(self) -> np.ndarray:
        return np.array([
            [np.cos(self.yaw), np.sin(self.yaw), -np.cos(self.yaw) * self.x - np.sin(self.yaw) * self.y],
            [-np.sin(self.yaw), np.cos(self.yaw), np.sin(self.yaw) * self.x - np.cos(self.yaw) * self.y],
            [0, 0, 1],
        ])

    @property
    def yaw_deg(self) -> float:
        return np.rad2deg(self.yaw)

    @staticmethod
    def from_matrix(M, time: float = 0) -> Pose:
        return Pose(x=M[0, -1], y=M[1, -1], yaw=np.arctan2(M[1, 0], M[0, 0]), time=time)

    def __str__(self) -> str:
        return f'{self.x:.3f}, {self.y:.3f}, {self.yaw_deg:.1f} deg'

    @overload
    def distance(self, other: Point) -> float: ...

    @overload
    def distance(self, other: Pose) -> float: ...

    def distance(self, other: Point | Pose) -> float:
        return float(np.sqrt((other.x - self.x)**2 + (other.y - self.y)**2))

    def projected_distance(self, other: Pose) -> float:
        return self.point.projected_distance(other.point, other.yaw)

    @overload
    def direction(self, other: Point) -> float: ...

    @overload
    def direction(self, other: Pose) -> float: ...

    def direction(self, other: Point | Pose) -> float:
        return float(np.arctan2(other.y - self.y, other.x - self.x))

    @overload
    def relative_direction(self, other: Point) -> float: ...

    @overload
    def relative_direction(self, other: Pose) -> float: ...

    def relative_direction(self, other: Point | Pose) -> float:
        return helpers.angle(self.yaw, self.direction(other))

    def __iadd__(self, step: PoseStep) -> Pose:
        self.x += step.linear * np.cos(self.yaw)
        self.y += step.linear * np.sin(self.yaw)
        self.yaw += step.angular
        self.time = step.time
        return self

    def __add__(self, step: PoseStep) -> Pose:
        return Pose(
            x=self.x + step.linear * np.cos(self.yaw),
            y=self.y + step.linear * np.sin(self.yaw),
            yaw=self.yaw + step.angular,
            time=step.time,
        )

    def transform(self, point: Point) -> Point:
        return Point(
            x=self.x + point.x * np.cos(self.yaw) - point.y * np.sin(self.yaw),
            y=self.y + point.x * np.sin(self.yaw) + point.y * np.cos(self.yaw),
        )

    def transform3d(self, point: Point3d) -> Point3d:
        return Point3d(
            x=self.x + point.x * np.cos(self.yaw) - point.y * np.sin(self.yaw),
            y=self.y + point.x * np.sin(self.yaw) + point.y * np.cos(self.yaw),
            z=point.z,
        )

    def transform_array(self, points: np.ndarray) -> np.ndarray:
        return np.stack((
            self.x + points[:, 0] * np.cos(self.yaw) - points[:, 1] * np.sin(self.yaw),
            self.y + points[:, 0] * np.sin(self.yaw) + points[:, 1] * np.cos(self.yaw),
        ), axis=1)

    def transform_pose(self, pose: Pose) -> Pose:
        return Pose(
            x=self.x + pose.x * np.cos(self.yaw) - pose.y * np.sin(self.yaw),
            y=self.y + pose.x * np.sin(self.yaw) + pose.y * np.cos(self.yaw),
            yaw=self.yaw + pose.yaw,
            time=pose.time,
        )

    def transform_line(self, line: Line) -> Line:
        a, b, c = self.inv_matrix.T @ line.tuple
        return Line(a=a, b=b, c=c)

    def relative_pose(self, other: Pose) -> Pose:
        return Pose.from_matrix(self.inv_matrix @ other.matrix)

    def relative_point(self, other: Point) -> Point:
        dx = other.x - self.x
        dy = other.y - self.y
        return Point(x=dx * np.cos(-self.yaw) - dy * np.sin(-self.yaw),
                     y=dx * np.sin(-self.yaw) + dy * np.cos(-self.yaw))

    def rotate(self, angle: float) -> Pose:
        return Pose(x=self.x, y=self.y, yaw=self.yaw+angle, time=self.time)

    def interpolate(self, other: Pose, t: float) -> Pose:
        return Pose(
            x=(1 - t) * self.x + t * other.x,
            y=(1 - t) * self.y + t * other.y,
            yaw=(1 - t) * self.yaw + t * other.yaw,
            time=(1 - t) * self.time + t * other.time,
        )
