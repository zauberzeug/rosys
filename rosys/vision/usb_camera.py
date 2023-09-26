from dataclasses import dataclass
from typing import Optional

from ..geometry import Rectangle
from .camera import Camera
from .image import ImageSize
from .image_rotation import ImageRotation


@dataclass(slots=True, kw_only=True)
class UsbCamera(Camera):
    active: bool = True
    detect: bool = False

    color: Optional[str] = None
    """a color code to identify the camera"""

    resolution: Optional[ImageSize] = None
    """physical resolution of the camera which should be used; camera may go into error state with wrong values"""

    auto_exposure: Optional[bool] = True
    """toggles auto exposure"""

    exposure: Optional[float] = None
    """manual exposure of the camera (between 0-1); set auto_exposure to False for this value to take effect"""

    rotation: ImageRotation = ImageRotation.NONE
    """rotation which should be applied after grabbing and cropping"""

    fps: Optional[int] = None
    """current frames per second (read only)"""

    crop: Optional[Rectangle] = None
    """region to crop on the original resolution before rotation"""

    @property
    def image_resolution(self) -> Optional[ImageSize]:
        if self.resolution is None:
            return None
        width = int(self.crop.width) if self.crop else self.resolution.width
        height = int(self.crop.height) if self.crop else self.resolution.height
        if self.rotation in {ImageRotation.LEFT, ImageRotation.RIGHT}:
            width, height = height, width
        return ImageSize(width=width, height=height)
