from typing import Protocol

from ..geometry import Pose


class PoseProvider(Protocol):

    @property
    def pose(self) -> Pose:
        ...
