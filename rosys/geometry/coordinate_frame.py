from __future__ import annotations

import logging
from dataclasses import dataclass, field
from uuid import uuid4

from .coordinate_frame_registry import coordinate_frame_registry
from .pose3d import Pose3d

logger = logging.getLogger('rosys.geometry.coordinate_frame')


@dataclass(slots=True, kw_only=True)
class CoordinateFrame(Pose3d):
    """A 3D coordinate frame based on a 3D pose.
    This is a simple wrapper around a Pose3d with an additional id.
    It guarantees that the frame is registered in the coordinate_frame_registry."""
    # id (uuid4 per default) of the frame
    id: str = field(default_factory=lambda: str(uuid4()))

    def __post_init__(self):
        coordinate_frame_registry[self.id] = self

    @classmethod
    def from_pose(cls, pose: Pose3d, id: str | None = None) -> CoordinateFrame:
        """Creates a new frame from a given pose.

        :param pose: The pose of the new frame.
        """
        id = id or str(uuid4())
        return cls(translation=pose.translation, rotation=pose.rotation, id=id,
                   parent_frame_id=pose.parent_frame_id)

    def delete(self) -> None:
        del coordinate_frame_registry[self.id]
        for frame in coordinate_frame_registry.values():
            if frame.parent_frame_id == self.id:
                logger.warning('Parent frame %s of %s was deleted', self.id, frame.id)
