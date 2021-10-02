from __future__ import annotations
from pydantic import BaseModel
import numpy as np


class Point3d(BaseModel):
    x: float
    y: float
    z: float

    @property
    def tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def distance(self, other: Point3d):
        squared_dist = np.sum((np.array(self.tuple) - np.array(other.tuple))**2, axis=0)
        return np.sqrt(squared_dist)
