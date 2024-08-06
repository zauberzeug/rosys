from __future__ import annotations

import abc
import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from typing_extensions import Self

from .point import Point
from .rotation import Rotation

frame_registry: dict[str, Frame3d] = {}


@dataclass(slots=True, kw_only=True)
class Object3d(abc.ABC):
    _frame_id: str | None = None

    @property
    def frame_id(self) -> str | None:
        return self._frame_id

    def in_frame(self, value: Frame3d | None) -> Self:
        self._frame_id = None if value is None else value.id
        return self

    def relative_to(self, target_frame: Pose3d | None) -> Self:
        """Compute the object location relative to the given frame"""
        if not self._frame_id and not target_frame:
            return self
        target_frame = target_frame or Pose3d.zero()
        source_frame = frame_registry[self._frame_id] if self._frame_id is not None else Pose3d.zero()
        return self.transform_with(target_frame.resolve().inverse() @ source_frame.resolve())

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


@dataclass(slots=True, kw_only=True)
class Point3d(Object3d):
    x: float
    y: float
    z: float

    @classmethod
    def zero(cls) -> Point3d:
        return cls(x=0, y=0, z=0)

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
