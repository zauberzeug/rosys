from __future__ import annotations
from pydantic import BaseModel
from typing import Optional
from .point import Point
from .line import Line


class LineSegment(BaseModel):

    point1: Point
    point2: Point

    @property
    def direction(self):
        return self.point1.direction(self.point2)

    @property
    def line(self):
        return Line.from_points(self.point1, self.point2)

    def intersect(self, other: LineSegment, allow_outside=False) -> Optional[Point]:
        # https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line_segment
        p1 = self.point1
        p2 = self.point2
        p3 = other.point1
        p4 = other.point2
        d = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x)
        s = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / d
        t = ((p2.x - p1.x) * (p1.y - p3.y) - (p2.y - p1.y) * (p1.x - p3.x)) / d
        if allow_outside or (0 <= s <= 1 and 0 <= t <= 1):
            return Point(x=p1.x + s * (p2.x - p1.x), y=p1.y + s * (p2.y - p1.y))

    def distance(self, point: Point):
        # https://stackoverflow.com/a/6853926/3419103
        a = point.x - self.point1.x
        b = point.y - self.point1.y
        c = self.point2.x - self.point1.x
        d = self.point2.y - self.point1.y
        dot = a * c + b * d
        len_sq = c * c + d * d
        t = dot / len_sq if len_sq > 0 else -1
        p = self.point1 if t < 0 else self.point2 if t > 1 else Point(x=self.point1.x + t * c, y=self.point1.y + t * d)
        return p.distance(point)
