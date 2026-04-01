from typing import Protocol

from nicegui import Event

from ..geometry import Pose


class PoseProvider(Protocol):
    """Protocol for objects that provide a 2D Pose."""
    POSE_UPDATED: Event[Pose]
    """Emitted when the pose has been updated."""

    @property
    def pose(self) -> Pose:
        ...
