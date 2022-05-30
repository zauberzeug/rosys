from __future__ import annotations

import math

from pydantic import BaseModel

from .point import Point


class Point3d(BaseModel):
    x: float
    y: float
    z: float

    @property
    def tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def distance(self, other: Point3d):
        return math.sqrt((other.x - self.x)**2 + (other.y - self.y)**2 + (other.z - self.z)**2)

    def projection(self) -> Point:
        return Point(x=self.x, y=self.y)

    def __str__(self) -> str:
        return f'Point3d({round(self.x, 2)}, {round(self.y, 2)}, {round(self.z, 2)})'

    def __add__(self, other: Point3d) -> Point3d:
        return Point3d(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)

    def __sub__(self, other: Point3d) -> Point3d:
        return Point3d(x=self.x - other.x, y=self.y - other.y, z=self.z - other.z)
