from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from .object3d import Object3d
from .point import Point

if TYPE_CHECKING:
    from .pose3d import Pose3d


@dataclass(slots=True, kw_only=True)
class Point3d(Object3d):
    x: float = 0
    y: float = 0
    z: float = 0

    @classmethod
    def zero(cls) -> Point3d:
        return Point3d(x=0, y=0, z=0)

    @staticmethod
    def from_tuple(t: Sequence[float]) -> Point3d:
        """Create a Point3d from the three first elements of a sequence."""
        return Point3d(x=t[0], y=t[1], z=t[2])

    @staticmethod
    def from_point(p: Point, z: float = 0) -> Point3d:
        """Create a Point3d from a Point."""
        return Point3d(x=p.x, y=p.y, z=z)

    @property
    def tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    @property
    def array(self) -> np.ndarray:
        return np.array(self.tuple).reshape(3, 1)

    def distance(self, other: Point3d) -> float:
        return math.sqrt((other.x - self.x)**2 + (other.y - self.y)**2 + (other.z - self.z)**2)

    def projection(self) -> Point:
        return Point(x=self.x, y=self.y)

    def __str__(self) -> str:
        return f'Point3d({self.x:.3f}, {self.y:.3f}, {self.z:.3f})'

    def __add__(self, other: Point3d) -> Point3d:
        return Point3d(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)

    def __sub__(self, other: Point3d) -> Point3d:
        return Point3d(x=self.x - other.x, y=self.y - other.y, z=self.z - other.z)

    def transform_with(self, pose: Pose3d) -> Point3d:
        """Transform this pose with another pose."""
        return Point3d.from_tuple((np.dot(pose.rotation.R, self.array) + pose.translation_vector).flatten())
