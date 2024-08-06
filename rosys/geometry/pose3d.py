from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass

import numpy as np
from typing_extensions import Self

from .frame3d_registry import frame_registry
from .object3d import Object3d
from .point3d import Point3d
from .rotation import Rotation


@dataclass(slots=True, kw_only=True)
class Pose3d(Object3d):
    """A 3D pose consisting of a translation and a rotation.

    The pose is stored as the transformation to the parent frame (not the objects in it).
    The transformation first applies the rotation and then the translation.
    """
    translation: Point3d
    rotation: Rotation

    def as_frame(self, frame_id: str) -> Frame3d:
        """Register this pose as a frame."""
        return Frame3d(_frame_id=self._frame_id, translation=self.translation, rotation=self.rotation, id=frame_id)

    @classmethod
    def zero(cls) -> Pose3d:
        return cls(translation=Point3d(x=0, y=0, z=0), rotation=Rotation.zero())

    @classmethod
    def from_matrix(cls, M: np.ndarray) -> Pose3d:
        return cls(translation=Point3d(x=M[0, 3] / M[3, 3], y=M[1, 3] / M[3, 3], z=M[2, 3] / M[3, 3]),
                   rotation=Rotation(R=(M[:3, :3] / M[3, 3]).tolist()))

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


@dataclass(slots=True, kw_only=True)
class Frame3d(Pose3d):
    id: str | None = None

    def __post_init__(self) -> None:
        if self.id is not None:
            frame_registry[self.id] = self

    def in_frame(self, frame: Frame3d | None) -> Self:
        if frame is not None and self in frame.ancestors:
            raise ValueError(f'Cannot place frame "{self.id}" in frame "{frame.id}" because it creates a cycle')
        self._frame_id = frame.id if frame is not None else None
        return self

    @property
    def ancestors(self) -> Iterator[Frame3d]:
        yield self
        if self.frame_id:
            yield from frame_registry[self.frame_id].ancestors
