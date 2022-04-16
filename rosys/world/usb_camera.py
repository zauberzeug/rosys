from enum import Enum
from typing import Optional

from pydantic import Field
from rosys.world.rectangle import Rectangle

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
    color: Optional[str] = Field(
        None,
        description='a color code to identify the camera'
    )
    resolution: Optional[ImageSize] = Field(
        None,
        description='physical resolution of the camera which should be used;'
        'camera may go into error state with wrong values'
    )
    exposure: Optional[float] = Field(
        None,
        description='required exposure between 0-1 or None for auto-exposure'
    )
    rotation: ImageRotation = Field(
        ImageRotation.NONE,
        description='rotation which should be applied after grabbing and cropping'
    )
    fps: Optional[int] = Field(
        None,
        description='current frames per second (read only)'
    )
    crop: Optional[Rectangle] = Field(
        None,
        description='region to crop on the original resolution before rotation'
    )

    @property
    def image_resolution(self) -> Optional[ImageSize]:
        if self.resolution is None:
            return None
        if self.rotation == ImageRotation.LEFT or self.rotation == ImageRotation.RIGHT:
            return ImageSize(width=self.resolution.height, height=self.resolution.width)
        else:
            return self.resolution.copy()
