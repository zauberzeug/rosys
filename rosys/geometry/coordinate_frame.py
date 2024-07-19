from __future__ import annotations

import logging
from uuid import uuid4

from .coordinate_frame_registry import coordinate_frame_registry
from .pose3d import Pose3d

logger = logging.getLogger('rosys.geometry.coordinate_frame')


class CoordinateFrame:

    def __init__(self, pose: Pose3d, id: str | None = None) -> None:
        if pose.frame_id in coordinate_frame_registry:
            raise ValueError(f'Frame with id {pose.frame_id} already exists. Use CoordinateFrame.from_pose() instead.')
        if pose.frame_id is None:
            pose.frame_id = id or str(uuid4())
        elif id and pose.frame_id != id:
            raise ValueError(f'Frame id "{id}" was specified, but pose already has a different id "{pose.frame_id}"')

        self.pose = pose

        coordinate_frame_registry[self.uuid] = self

    @property
    def uuid(self) -> str:
        assert self.pose.frame_id is not None
        return self.pose.frame_id

    @classmethod
    def from_pose(cls, pose: Pose3d) -> CoordinateFrame:
        """Creates a new frame from a given pose.

        :param pose: The pose of the new frame.
        :param relative_to: The parent frame of the new frame. (default: None = world frame)
        """
        if pose.frame_id in coordinate_frame_registry:
            return coordinate_frame_registry[pose.frame_id]
        logger.debug('Creating new frame from pose %s', pose)
        return cls(pose=pose)

    @property
    def parent_uuid(self) -> str | None:
        return self.pose.parent_frame_id

    def resolve(self, pose: Pose3d | None, relative_to: CoordinateFrame | str | None = None) -> Pose3d:
        """Recursively resolves the pose to the given parent frame.

        :param pose: The pose to resolve. (default: zero pose)
        :param up_to: The parent frame to resolve the pose to. (default: None = world frame)
        """
        if not pose:
            pose = Pose3d.zero()

        if isinstance(relative_to, str):
            try:
                relative_to = coordinate_frame_registry[relative_to]
            except KeyError as e:
                raise ValueError(f'Could not resolve pose to {relative_to} (Frame does not exist)') from e

        if self.parent_uuid is None:
            return self.pose * pose

        final_pose = Pose3d(translation=pose.translation, rotation=pose.rotation)
        current_frame: CoordinateFrame | None = self
        while current_frame != relative_to:
            if current_frame is None:
                raise ValueError(f'Could not resolve pose to {relative_to} (end of chain)')

            final_pose = current_frame.pose * final_pose
            current_frame = coordinate_frame_registry[current_frame.parent_uuid] if current_frame.parent_uuid else None

        final_pose.parent_frame = relative_to
        return final_pose

    def delete(self) -> None:
        del coordinate_frame_registry[self.uuid]
        for frame in coordinate_frame_registry.values():
            if frame.parent_uuid == self.uuid:
                logger.warning('Parent frame %s of %s was deleted', self.uuid, frame.uuid)
