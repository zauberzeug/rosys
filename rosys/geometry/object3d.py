from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import TYPE_CHECKING

from typing_extensions import Self

from .frame3d_registry import frame_registry

if TYPE_CHECKING:
    from .pose3d import Frame3d, Pose3d


@dataclass(slots=True, kw_only=True)
class Object3d(abc.ABC):
    _frame_id: str | None = None

    @property
    def frame_id(self) -> str | None:
        return self._frame_id

    def in_frame(self, frame: Frame3d | None) -> Self:
        self._frame_id = None if frame is None else frame.id
        return self

    def relative_to(self, target_frame: Pose3d | None) -> Self:
        """Compute the object location relative to the given frame pose.

        The ``frame_id`` of the resulting object will be None.
        """
        if self._frame_id is None:
            if target_frame is None:
                return self
            else:
                return self.transform_with(target_frame.resolve().inverse())
        else:  # noqa: PLR5501
            if target_frame is None:
                return self.transform_with(frame_registry[self._frame_id].resolve())
            else:
                return self.transform_with(target_frame.resolve().inverse() @ frame_registry[self._frame_id].resolve())

    def resolve(self) -> Self:
        """Compute the object location relative to the world frame."""
        return self.relative_to(None)

    @abc.abstractmethod
    def transform_with(self, pose: Pose3d) -> Self:
        pass
