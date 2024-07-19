
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .coordinate_frame import CoordinateFrame

coordinate_frame_registry: dict[str, 'CoordinateFrame'] = {}
