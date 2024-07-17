from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .point import Point


@dataclass(slots=True, kw_only=True)
class Point3d(Sequence[float]):
    x: float
    y: float
    z: float

    @staticmethod
    def from_tuple(t: Sequence[float]) -> Point3d:
        """Create a Point3d from the three first elements of a sequence."""
        return Point3d(x=t[0], y=t[1], z=t[2])

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

    # sequence

    def __len__(self) -> int:
        return 3

    def __getitem__(self, index: int | slice) -> float | tuple[float]:
        return self.tuple[index]

    def __iter__(self):
        return iter(self.tuple)
