from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .frame3d import Frame3d

coordinate_frame_registry: dict[str, Frame3d] = {}
