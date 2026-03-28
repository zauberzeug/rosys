from typing import Protocol

from nicegui import Event

from ..geometry import Pose


class PoseProvider(Protocol):
    POSE_UPDATED: Event[Pose]

    @property
    def pose(self) -> Pose:
        ...
