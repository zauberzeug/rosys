from __future__ import annotations

from dataclasses import dataclass
from uuid import uuid4

from .pose3d import Pose3d

registry: dict[str, Pose3d] = {}


@dataclass(slots=True, init=False)
class Frame3d:
    """A 3D coordinate frame based on a 3D pose.

    This is a simple wrapper around a Pose3d with an additional ID.
    It guarantees that the frame is registered in the coordinate_frame_registry.
    """
    id: str
    _pose: Pose3d
    _parent: Frame3d | None = None

    def __init__(self,
                 pose: Pose3d,
                 parent: Frame3d | None = None,
                 id: str | None = None,  # pylint: disable=redefined-builtin
                 ) -> None:
        self.id = id or str(uuid4())
        self.pose = pose
        self.parent = parent

    @property
    def pose(self) -> Pose3d:
        return registry[self.id]

    @pose.setter
    def pose(self, value: Pose3d) -> None:
        registry[self.id] = value

    @property
    def parent(self) -> Frame3d | None:
        return self._parent

    @parent.setter
    def parent(self, value: Frame3d | None) -> None:
        self._parent = value
        self._check_for_cycles()

    def _check_for_cycles(self) -> None:
        """Checks for cycles in the frame hierarchy."""
        frame: Frame3d = self
        visited = {self.id}
        while frame.parent:
            frame = frame.parent
            if frame.id in visited:
                raise ValueError('Setting parent frame would create a cycle in the frame hierarchy')
            visited.add(frame.id)

    @property
    def world_pose(self) -> Pose3d:
        """The pose of this frame relative to the world frame."""
        return Pose3d.zero() if self.parent is None else self.parent.world_pose @ self.pose
