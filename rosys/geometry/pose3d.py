from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from .object3d import Object3d
from .point3d import Point3d
from .rotation import Rotation

if TYPE_CHECKING:
    from .frame3d import Frame3d


@dataclass(slots=True, kw_only=True)
class Pose3d(Object3d):
    """A 3D pose consisting of a translation and a rotation.

    The pose is stored as the transformation to the parent frame (not the objects in it).
    The transformation first applies the rotation and then the translation.
    """
    translation: Point3d
    rotation: Rotation

    @classmethod
    def zero(cls, *, frame: Frame3d | None = None) -> Pose3d:
        return cls(translation=Point3d(x=0, y=0, z=0), rotation=Rotation.zero(), frame=frame)

    @classmethod
    def from_matrix(cls, M: np.ndarray, *, frame: Frame3d | None = None) -> Pose3d:
        return cls(translation=Point3d(x=M[0, 3] / M[3, 3], y=M[1, 3] / M[3, 3], z=M[2, 3] / M[3, 3]),
                   rotation=Rotation(R=(M[:3, :3] / M[3, 3]).tolist()),
                   frame=frame)

    @property
    def matrix(self) -> np.ndarray:
        return np.block([[self.rotation.matrix, self.translation.array], [0, 0, 0, 1]])

    @property
    def inverse_matrix(self) -> np.ndarray:
        return np.block([[self.rotation.T.matrix, -self.rotation.matrix.T @ self.translation.array], [0, 0, 0, 1]])

    def inverse(self) -> Pose3d:
        return Pose3d.from_matrix(self.inverse_matrix)

    def __matmul__(self, other: Pose3d) -> Pose3d:
        return Pose3d.from_matrix(self.matrix @ other.matrix)

    def __str__(self) -> str:
        return f'R = {self.rotation}\nT = {self.translation}'

    def transform_with(self, pose: Pose3d) -> Pose3d:
        """Transform this pose with another pose."""
        return pose @ self
