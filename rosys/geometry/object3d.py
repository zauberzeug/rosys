from __future__ import annotations

import abc
import math
from collections import defaultdict
from collections.abc import Sequence
from dataclasses import dataclass
from uuid import uuid4

import numpy as np
from typing_extensions import Self

from .point import Point
from .rotation import Rotation


@dataclass(slots=True, init=False)
class Frame3d:
    """A 3D coordinate frame based on a 3D pose.

    This is a simple wrapper around a Pose3d with an additional ID.
    It guarantees that the frame is registered in the coordinate_frame_registry.
    """
    id: str

    def __init__(self,
                 pose: Pose3d | None = None,
                 id: str | None = None,  # pylint: disable=redefined-builtin
                 ) -> None:
        self.id = id or str(uuid4())
        if pose is not None:
            self.pose = pose

    @property
    def pose(self) -> Pose3d:
        return registry[self.id]

    @pose.setter
    def pose(self, value: Pose3d) -> None:
        registry[self.id] = value

    # def _check_for_cycles(self) -> None:
    #     """Checks for cycles in the frame hierarchy."""
    #     frame: Frame3d = self
    #     visited = {self.id}
    #     while frame.parent:
    #         frame = frame.parent
    #         if frame.id in visited:
    #             raise ValueError('Setting parent frame would create a cycle in the frame hierarchy')
    #         visited.add(frame.id)

    @property
    def world_pose(self) -> Pose3d:
        """The pose of this frame relative to the world frame."""
        return self.pose.resolve()


@dataclass(slots=True, kw_only=True)
class Object3d(abc.ABC):
    frame: Frame3d | None = None

    def relative_to(self, target_frame: Frame3d | None) -> Self:
        """Compute the object location relative to the given frame"""
        if target_frame and self.frame:
            return self.transform_with(target_frame.world_pose.inverse() @ self.frame.world_pose)
        elif target_frame:
            return self.transform_with(target_frame.world_pose.inverse())
        elif self.frame:
            return self.transform_with(self.frame.world_pose)
        else:
            return self

    def resolve(self) -> Self:
        """Compute the object location relative to the world frame."""
        return self.relative_to(None)

    @abc.abstractmethod
    def transform_with(self, pose: Pose3d) -> Self:
        pass


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


@dataclass(slots=True, kw_only=True)
class Point3d(Object3d):
    x: float
    y: float
    z: float

    @classmethod
    def zero(cls, *, frame: Frame3d | None = None) -> Point3d:
        return cls(x=0, y=0, z=0, frame=frame)

    @staticmethod
    def from_tuple(t: Sequence[float]) -> Point3d:
        """Create a Point3d from the three first elements of a sequence."""
        return Point3d(x=t[0], y=t[1], z=t[2])

    @staticmethod
    def from_point(p: Point, z: float = 0) -> Point3d:
        """Create a Point3d from a Point."""
        return Point3d(x=p.x, y=p.y, z=z)

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

    def transform_with(self, pose: Pose3d) -> Point3d:
        """Transform this pose with another pose."""
        return Point3d.from_tuple(np.dot(pose.rotation.R, self.array) + pose.translation.array)


registry: defaultdict[str, Pose3d] = defaultdict(Pose3d.zero)
