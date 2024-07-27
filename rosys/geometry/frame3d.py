from __future__ import annotations

import logging
from dataclasses import dataclass, field
from uuid import uuid4

from .coordinate_frame_registry import coordinate_frame_registry
from .pose3d import Pose3d

logger = logging.getLogger('rosys.geometry.coordinate_frame')


@dataclass(slots=True, kw_only=True)
class Frame3d(Pose3d):
    """A 3D coordinate frame based on a 3D pose.

    This is a simple wrapper around a Pose3d with an additional ID.
    It guarantees that the frame is registered in the coordinate_frame_registry.
    """
    id: str = field(default_factory=lambda: str(uuid4()))

    def __post_init__(self) -> None:
        coordinate_frame_registry[self.id] = self

    @property
    def parent_frame_id(self) -> str | None:
        return self._parent_frame_id

    @parent_frame_id.setter
    def parent_frame_id(self, value: str | None) -> None:
        previous_parent_id = self._parent_frame_id
        self._parent_frame_id = value
        if self._check_for_cycles():
            self._parent_frame_id = previous_parent_id
            raise ValueError('Setting parent frame would create a cycle in the frame hierarchy')

    def _check_for_cycles(self) -> bool:
        """Checks for cycles in the parent_frame hierarchy."""
        frame: Frame3d | None = self
        visited = set()
        while frame:
            if frame.id in visited:
                return True
            visited.add(frame.id)
            frame = frame.parent_frame

        return False

    @classmethod
    def from_pose(cls, pose: Pose3d, id: str | None = None) -> Frame3d:  # pylint: disable=redefined-builtin
        """Creates a new frame from a given pose.

        :param pose: The pose of the new frame.
        """
        id = id or str(uuid4())
        instance = cls(translation=pose.translation, rotation=pose.rotation, id=id)
        instance.parent_frame = pose.parent_frame
        return instance

    def delete(self) -> None:
        """Remove the frame from the registry."""
        del coordinate_frame_registry[self.id]
        for frame in coordinate_frame_registry.values():
            if frame.parent_frame_id == self.id:
                logger.warning('Parent frame %s of %s was deleted', self.id, frame.id)

    @staticmethod
    def common_frame(pose1: Pose3d | None, pose2: Pose3d | None) -> Frame3d | None:
        """Returns the first common frame of a pair of poses or frames.

        :param pose1: The first pose or frame.
        :param pose2: The second pose or frame.
        """
        frame1 = pose1 if isinstance(pose1, Frame3d) else pose1.parent_frame if pose1 else None
        frame2 = pose2 if isinstance(pose2, Frame3d) else pose2.parent_frame if pose2 else None
        return Frame3d._find_common_frame(frame1, frame2)
