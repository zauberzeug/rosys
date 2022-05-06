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
    active: bool = True
    detect: bool = False
    color: Optional[str] = Field(
        None,
        description='a color code to identify the camera'
    )
    resolution: Optional[ImageSize] = Field(
        None,
        description='physical resolution of the camera which should be used;'
        'camera may go into error state with wrong values'
    )
    auto_exposure: Optional[bool] = Field(
        True,
        description='toggles auto exposure'
    )
    exposure: Optional[float] = Field(
        None,
        description='manual exposure of the camera (between 0-1); set auto_exposure to False for this value to take effect'
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
        width = self.crop.width if self.crop else self.resolution.width
        height = self.crop.height if self.crop else self.resolution.height
        if self.rotation == ImageRotation.LEFT or self.rotation == ImageRotation.RIGHT:
            return ImageSize(width=height, height=width)
        else:
            return self.resolution.copy()
