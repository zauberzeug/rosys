from enum import Enum
from typing import Optional

from .camera import Camera
from .image import ImageSize


class ImageRotation(str, Enum):
    NONE: int = 0
    RIGHT: int = 90
    UPSIDE_DOWN: int = 180
    LEFT: int = 270


class UsbCamera(Camera):
    capture: bool = True
    detect: bool = False
    connected: bool = False
    resolution: Optional[ImageSize] = None
    exposure: Optional[float] = None
    color: Optional[str] = None
    rotation: ImageRotation = ImageRotation.NONE

    @property
    def image_resolution(self) -> Optional[ImageSize]:
        if self.resolution is None:
            return None
        if self.rotation == ImageRotation.LEFT or self.rotation == ImageRotation.RIGHT:
            return ImageSize(width=self.resolution.height, height=self.resolution.width)
        else:
            return self.resolution.copy()
