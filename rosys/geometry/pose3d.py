from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from typing_extensions import Self

from .coordinate_frame_registry import coordinate_frame_registry
from .point3d import Point3d
from .rotation import Rotation

if TYPE_CHECKING:
    from .frame3d import Frame3d


@dataclass(slots=True, init=False)
class Pose3d:
    """A 3D pose consisting of a translation and a rotation.

    The pose is stored as the transformation to the parent frame (not the objects in it).
    The transformation first applies the rotation and then the translation.
    """
    translation: Point3d
    rotation: Rotation
    _parent_frame_id: str | None = None

    def __init__(self, translation: Point3d, rotation: Rotation, parent_frame: Frame3d | None = None) -> None:
        self.translation = translation
        self.rotation = rotation
        self._parent_frame_id = parent_frame.id if parent_frame else None

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

    @property
    def parent_frame_id(self) -> str | None:
        return self._parent_frame_id

    @parent_frame_id.setter
    def parent_frame_id(self, value: str | None) -> None:
        self._parent_frame_id = value

    @property
    def parent_frame(self) -> Frame3d | None:
        return coordinate_frame_registry[self.parent_frame_id] if self.parent_frame_id else None

    @parent_frame.setter
    def parent_frame(self, value: Frame3d | None) -> None:
        self.parent_frame_id = value.id if value else None

    def rotate(self, rotation: Rotation) -> Self:
        self.rotation *= rotation
        return self

    def translate(self, translation: Point3d) -> Self:
        self.translation += translation
        return self

    def relative_to(self, target_frame: Frame3d | None) -> Pose3d:
        """Compute the pose relative to the given ancestor frame.

        :param target_frame: The ancestor frame to resolve the pose to (default: ``None`` for the world frame)
        """
        world_self = self.parent_frame.world_pose @ self if self.parent_frame else self
        world_target = target_frame.world_pose if target_frame else Pose3d.zero()
        return world_target.inverse() @ world_self

    def resolve(self) -> Pose3d:
        return self.relative_to(None)

    def inverse(self) -> Pose3d:
        return Pose3d.from_matrix(self.inverse_matrix)

    def __matmul__(self, other: Pose3d) -> Pose3d:
        return Pose3d.from_matrix(self.matrix @ other.matrix)

    def __str__(self) -> str:
        return f'R = {self.rotation}\nT = {self.translation}'
