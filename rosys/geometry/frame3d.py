from __future__ import annotations

import logging
from dataclasses import dataclass
from uuid import uuid4

from .coordinate_frame_registry import coordinate_frame_registry
from .point3d import Point3d
from .pose3d import Pose3d
from .rotation import Rotation

logger = logging.getLogger('rosys.geometry.coordinate_frame')


@dataclass(init=False)
class Frame3d(Pose3d):
    """A 3D coordinate frame based on a 3D pose.

    This is a simple wrapper around a Pose3d with an additional ID.
    It guarantees that the frame is registered in the coordinate_frame_registry.
    """
    id: str = ''

    def __init__(self,
                 translation: Point3d,
                 rotation: Rotation,
                 parent_frame: Frame3d | None = None,
                 id: str | None = None,  # pylint: disable=redefined-builtin
                 ) -> None:
        super().__init__(translation, rotation, parent_frame)
        self.id = id or str(uuid4())
        assert self.id not in coordinate_frame_registry, f'Frame with ID {self.id} already exists'
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
        :param id: Optional ID of the new frame. If not given, a random ID is generated.
        """
        return cls(translation=pose.translation, rotation=pose.rotation, parent_frame=pose.parent_frame, id=id)

    def delete(self) -> None:
        """Remove the frame from the registry."""
        del coordinate_frame_registry[self.id]
        for frame in coordinate_frame_registry.values():
            if frame.parent_frame_id == self.id:
                logger.warning('Parent frame %s of %s was deleted', self.id, frame.id)

    @property
    def world_pose(self) -> Pose3d:
        """The pose of this frame relative to the world frame."""
        return Pose3d.zero() if self.parent_frame is None else self.parent_frame.world_pose @ self
