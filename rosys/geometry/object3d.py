from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import TYPE_CHECKING

from typing_extensions import Self

if TYPE_CHECKING:
    from .frame3d import Frame3d
    from .pose3d import Pose3d


@dataclass(slots=True, kw_only=True)
class Object3d(abc.ABC):
    frame: Frame3d | None = None

    def relative_to(self, target_frame: Frame3d | None) -> Self:
        """Compute the object location relative to the given frame"""
        if target_frame and self.frame:
            return self.transform_with(target_frame.world_pose.inverse() @ self.frame.pose)
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
