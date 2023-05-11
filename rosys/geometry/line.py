from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .point import Point


def skew(a, b, c):
    return np.array([[0, -c, b], [c, 0, -a], [-b, a, 0]])


@dataclass(slots=True, kw_only=True)
class Line:
    a: float
    b: float
    c: float

    @property
    def tuple(self) -> tuple[float, float, float]:
        return (self.a, self.b, self.c)

    @property
    def yaw(self) -> float:
        return np.arctan2(self.b, self.a) - np.pi / 2

    @staticmethod
    def from_points(point1: Point, point2: Point) -> Line:
        l = skew(point1.x, point1.y, 1) @ [point2.x, point2.y, 1]
        return Line(a=l[0], b=l[1], c=l[2])

    def intersect(self, other: Line) -> Point:
        p = np.cross(self.tuple, other.tuple)
        return Point(x=p[0] / p[-1], y=p[1] / p[-1])

    def foot_point(self, point: Point) -> Point:
        G3 = np.diag([1, 1, 0])
        p = skew(self.a, self.b, self.c) @ skew(point.x, point.y, 1) @ G3 @ self.tuple
        return Point(x=p[0] / p[-1], y=p[1] / p[-1])

    def distance(self, point: Point) -> float:
        return abs((self.a * point.x + self.b * point.y + self.c) / np.sqrt(self.a**2 + self.b**2))
