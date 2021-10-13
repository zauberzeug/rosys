from __future__ import annotations
from pydantic import BaseModel
import math


class Point3d(BaseModel):
    x: float
    y: float
    z: float

    @property
    def tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def distance(self, other: Point3d):
        return math.sqrt((other.x - self.x)**2 + (other.y - self.y)**2 + (other.z - self.z)**2)

    def __str__(self) -> str:
        return f'Point({round(self.x, 2)}, {round(self.y, 2)}, {round(self.z, 2)})'
