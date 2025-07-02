from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .point3d import Point3d


@dataclass(slots=True, kw_only=True)
class Line3d:
    origin: Point3d
    direction: Point3d

    @property
    def array(self) -> np.ndarray:
        """Return the line as a (6,) numpy array: [origin, direction]."""
        return np.concatenate([self.origin.array.flatten(), self.direction.array.flatten()])

    @staticmethod
    def from_array(array: np.ndarray) -> Line3d:
        """Create a Line3d from a (6,) numpy array: [origin, direction]."""
        return Line3d(origin=Point3d.from_array(array[:3]), direction=Point3d.from_array(array[3:]))
