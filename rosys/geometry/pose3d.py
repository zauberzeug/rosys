from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .point3d import Point3d
from .rotation import Rotation


@dataclass(slots=True, kw_only=True)
class Pose3d:
    '''A 3D pose consisting of a translation and a rotation.
    The pose is stored as the transformation to the parent frame (not the objects in it).
    The transformation first applies the rotation and then the translation.
    '''
    translation: Point3d
    rotation: Rotation
    parent_frame_pose: Optional['Pose3d'] = None

    @classmethod
    def zero(cls) -> 'Pose3d':
        return cls(translation=Point3d(x=0, y=0, z=0), rotation=Rotation.zero())

    @classmethod
    def from_matrix(cls, M: np.ndarray) -> 'Pose3d':
        return cls(translation=Point3d(x=M[0, 3], y=M[1, 3], z=M[2, 3]), rotation=Rotation(R=M[:3, :3].tolist()))

    @property
    def matrix(self) -> np.ndarray:
        return np.block([[self.rotation.matrix, self.translation.array], [0, 0, 0, 1]])

    @property
    def inverse_matrix(self) -> np.ndarray:
        return np.block([[self.rotation.T, -self.rotation.matrix.T @ self.translation.array], [0, 0, 0, 1]])

    def rotate(self, rotation: Rotation):
        self.rotation *= rotation
        return self

    def translate(self, translation: Point3d):
        self.translation += translation
        return self

    def transform_point(self, point: Point3d) -> Point3d:
        return Point3d.from_tuple(self.rotation.matrix @ point.array + self.translation.array)

    def relative_to(self, other: Pose3d) -> Pose3d:
        return Pose3d(translation=Point3d.from_tuple(self.rotation.T @ (self.translation - other.translation).array), rotation=self.rotation * other.rotation.T)

    def resolve(self, up_to: Optional[Pose3d] = None) -> Pose3d:
        '''Recursively resolves the pose to the given parent frame.

        :param up_to: The parent frame to resolve the pose to. (default: None = world frame)
        '''
        if self.parent_frame_pose is None or self.parent_frame_pose == up_to:
            return self

        final_pose: Pose3d = Pose3d(translation=self.translation, rotation=self.rotation)
        current_frame: Optional[Pose3d] = self.parent_frame_pose
        while current_frame != up_to:
            if current_frame is None:
                raise ValueError(f'Could not resolve pose to {up_to} (end of chain)')

            final_pose = current_frame * final_pose
            current_frame = current_frame.parent_frame_pose

        final_pose.parent_frame_pose = up_to
        return final_pose

    def __mul__(self, other: 'Pose3d') -> Pose3d:
        return Pose3d.from_matrix(self.matrix @ other.matrix)

    def __str__(self) -> str:
        return f'R = {self.rotation}\nT = {self.translation}'
