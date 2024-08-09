from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .pose3d import Frame3d

frame_registry: dict[str, Frame3d] = {}
