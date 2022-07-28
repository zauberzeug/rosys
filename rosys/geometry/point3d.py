from __future__ import annotations

import math
from dataclasses import dataclass

from .point import Point


@dataclass(slots=True, kw_only=True)
class Point3d:
    x: float
    y: float
    z: float

    @property
    def tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def distance(self, other: Point3d) -> float:
        return math.sqrt((other.x - self.x)**2 + (other.y - self.y)**2 + (other.z - self.z)**2)

    def projection(self) -> Point:
        return Point(x=self.x, y=self.y)

    def __str__(self) -> str:
        return f'Point3d({round(self.x, 2)}, {round(self.y, 2)}, {round(self.z, 2)})'

    def __add__(self, other: Point3d) -> Point3d:
        return Point3d(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)

    def __sub__(self, other: Point3d) -> Point3d:
        return Point3d(x=self.x - other.x, y=self.y - other.y, z=self.z - other.z)
