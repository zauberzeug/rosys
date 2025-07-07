from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, overload

import numpy as np

if TYPE_CHECKING:
    from .pose import Pose


@dataclass(slots=True, kw_only=True)
class Point:
    x: float
    y: float

    @staticmethod
    def from_complex(number: complex) -> Point:
        return Point(
            x=np.real(number),
            y=np.imag(number),
        )

    @property
    def tuple(self) -> tuple[float, float]:
        return (self.x, self.y)

    @property
    def complex(self) -> complex:
        return self.x + 1j * self.y

    @overload
    def distance(self, other: Point) -> float: ...

    @overload
    def distance(self, other: Pose) -> float: ...

    def distance(self, other: Point | Pose) -> float:
        return float(np.sqrt((other.x - self.x)**2 + (other.y - self.y)**2))

    @overload
    def direction(self, other: Point) -> float: ...

    @overload
    def direction(self, other: Pose) -> float: ...

    def direction(self, other: Point | Pose) -> float:
        return float(np.arctan2(other.y - self.y, other.x - self.x))

    def projected_distance(self, other: Point, direction: float) -> float:
        def d(p):
            return np.sqrt(p.x**2 + p.y**2) * np.cos(direction - np.arctan2(p.y, p.x))
        return float(d(other) - d(self))

    def polar(self, distance: float, yaw: float) -> Point:
        return Point(
            x=self.x + distance * np.cos(yaw),
            y=self.y + distance * np.sin(yaw),
        )

    def interpolate(self, other: Point, t: float) -> Point:
        return Point(
            x=(1 - t) * self.x + t * other.x,
            y=(1 - t) * self.y + t * other.y,
        )

    def __add__(self, other: Point) -> Point:
        return Point(x=self.x + other.x, y=self.y + other.y)

    def __sub__(self, other: Point) -> Point:
        return Point(x=self.x - other.x, y=self.y - other.y)

    def __mul__(self, factor) -> Point:
        return Point(x=self.x * factor, y=self.y * factor)

    def __truediv__(self, factor) -> Point:
        return Point(x=self.x / factor, y=self.y / factor)

    def __str__(self) -> str:
        return f'Point({self.x:.3f}, {self.y:.3f})'
