from __future__ import annotations

from dataclasses import dataclass

# imports for type annotations only
from typing import TYPE_CHECKING, Optional

import numpy as np

from .point3d import Point3d
from .rotation import Rotation

if TYPE_CHECKING:
    from .coordinate_frame import CoordinateFrame

from .coordinate_frame_registry import coordinate_frame_registry


@dataclass(slots=True, kw_only=True)
class Pose3d:
    """A 3D pose consisting of a translation and a rotation.
    The pose is stored as the transformation to the parent frame (not the objects in it).
    The transformation first applies the rotation and then the translation.
    """
    translation: Point3d
    rotation: Rotation
    parent_frame_id: str | None = None

    @classmethod
    def zero(cls) -> Pose3d:
        return cls(translation=Point3d(x=0, y=0, z=0), rotation=Rotation.zero())

    @classmethod
    def from_matrix(cls, M: np.ndarray) -> Pose3d:
        return cls(translation=Point3d(x=M[0, 3], y=M[1, 3], z=M[2, 3]), rotation=Rotation(R=M[:3, :3].tolist()))

    @property
    def matrix(self) -> np.ndarray:
        return np.block([[self.rotation.matrix, self.translation.array], [0, 0, 0, 1]])

    @property
    def inverse_matrix(self) -> np.ndarray:
        return np.block([[self.rotation.T, -self.rotation.matrix.T @ self.translation.array], [0, 0, 0, 1]])

    @property
    def parent_frame(self) -> CoordinateFrame | None:
        return coordinate_frame_registry[self.parent_frame_id] if self.parent_frame_id else None

    @parent_frame.setter
    def parent_frame(self, value: CoordinateFrame | None):
        self.parent_frame_id = value.id if value else None

    def rotate(self, rotation: Rotation):
        self.rotation *= rotation
        return self

    def translate(self, translation: Point3d):
        self.translation += translation
        return self

    def resolve(self, up_to: Optional[CoordinateFrame] = None) -> Pose3d:
        '''Recursively resolves the pose to the given parent frame.

        :param up_to: The parent frame to resolve the pose to. (default: None = world frame)
        '''
        parent_frame = self.parent_frame
        if parent_frame is None or parent_frame == up_to:
            return self

        final_pose: Pose3d = Pose3d(translation=self.translation, rotation=self.rotation)
        current_frame: Optional[Pose3d] = parent_frame
        while current_frame != up_to:
            if current_frame is None:
                raise ValueError(f'Could not resolve pose to {up_to} (end of chain)')

            final_pose = current_frame @ final_pose
            current_frame = current_frame.parent_frame

        final_pose.parent_frame = up_to
        return final_pose

    def transform_point(self, point: Point3d, source_frame: CoordinateFrame | None = None) -> Point3d:
        '''Transforms the coordinates of a point from the source frame to the current frame.

        :param point: The point to transform.
        :param source_frame: The frame the point is currently in. (default: None = world frame)
        '''
        if source_frame is None:
            relative_pose = self.resolve(up_to=source_frame)
        else:
            relative_pose = self.relative_to(source_frame, common_frame=source_frame)
        return Point3d.from_tuple((relative_pose.rotation.matrix.T @ (point - relative_pose.translation)).tolist())

    def relative_to(self, other: Pose3d, common_frame: CoordinateFrame | None = None) -> Pose3d:
        '''Calculates the relative pose of this pose to another pose.

        :param other: The other pose.
        :param common_frame: The common parent frame to resolve the poses to. (default: None = world frame)
        '''
        resolved_self = self.resolve(up_to=common_frame)
        resolved_other = other.resolve(up_to=common_frame)

        return resolved_other.inverse() @ resolved_self

    @staticmethod
    def common_frame(pose1: Pose3d, pose2: Pose3d) -> CoordinateFrame | None:
        '''Finds the first common parent frame of two poses.

        :param pose1: The first pose.
        :param pose2: The second pose.
        '''
        return Pose3d._find_common_frame(pose1.parent_frame, pose2.parent_frame)

    @staticmethod
    def _find_common_frame(frame1: CoordinateFrame | None, frame2: CoordinateFrame | None) -> CoordinateFrame | None:
        '''Finds the first common parent frame of two frames.

        :param frame1: The first frame.
        :param frame2: The second frame.
        '''
        chain2 = []
        while frame2 is not None:
            chain2.append(frame2)
            frame2 = frame2.parent_frame
        while frame1 is not None:
            if frame1 in chain2:
                return frame1
            frame1 = frame1.parent_frame

        return None

    def inverse(self) -> Pose3d:
        return Pose3d(translation=Point3d.from_tuple((-self.rotation.T.matrix @ self.translation).tolist()), rotation=self.rotation.T)

    def __matmul__(self, other: Pose3d) -> Pose3d:
        # if not self.parent_frame_id == other.parent_frame_id:
        #     raise ValueError(
        #         'Cannot multiply poses in different parent frames. Use relative_to() or resolve() to bring them into a common frame.')
        new_pose = Pose3d.from_matrix(self.matrix @ other.matrix)
        # new_pose.parent_frame_id = self.parent_frame_id
        return new_pose

    def __str__(self) -> str:
        return f'R = {self.rotation}\nT = {self.translation}'

    def __eq__(self, value: object) -> bool:
        if not isinstance(value, Pose3d):
            return False
        return self.translation == value.translation and self.rotation == value.rotation and self.parent_frame_id == value.parent_frame_id
