from dataclasses import dataclass
from enum import Enum
from typing import Optional

import rosys

from ..geometry import Rectangle
from .camera import Camera
from .image import ImageSize


class ImageRotation(str, Enum):
    NONE: int = 0
    RIGHT: int = 90
    UPSIDE_DOWN: int = 180
    LEFT: int = 270

    @classmethod
    def from_int(cls, degrees: int) -> 'ImageRotation':
        degrees_normalized = degrees % 360
        return min(cls, key=lambda rotation: abs(degrees_normalized - int(rotation)))


@dataclass(slots=True, kw_only=True)
class RtspCamera(Camera):
    active: bool = True
    detect: bool = False
    url: Optional[str] = None

    color: Optional[str] = None
    '''a color code to identify the camera'''

    resolution: Optional[ImageSize] = None
    '''physical resolution of the camera which should be used; camera may go into error state with wrong values'''

    rotation: ImageRotation = ImageRotation.NONE
    '''rotation which should be applied after grabbing and cropping'''

    fps: Optional[int] = None
    '''current frames per second (read only)'''

    crop: Optional[Rectangle] = None
    '''region to crop on the original resolution before rotation'''

    @property
    def image_resolution(self) -> Optional[ImageSize]:
        if self.resolution is None:
            return None
        width = self.crop.width if self.crop else self.resolution.width
        height = self.crop.height if self.crop else self.resolution.height
        if self.rotation == ImageRotation.LEFT or self.rotation == ImageRotation.RIGHT:
            return ImageSize(width=height, height=width)
        else:
            return ImageSize(width=self.resolution.width, height=self.resolution.height)

    @property
    def rotation_angle(self) -> int:
        return int(self.rotation)

    @rotation_angle.setter
    def rotation_angle(self, value: int) -> None:
        self.rotation = ImageRotation.from_int(value)

    def rotate_clockwise(self) -> None:
        self.rotation_angle += 90

    def rotate_counter_clockwise(self) -> None:
        self.rotation_angle -= 90

    def __repr__(self) -> dict:
        last_image = f'{rosys.time() - self.latest_captured_image.time:.0f} s ago' if self.latest_captured_image else 'never'
        return {'name': self.name, 'url': self.url, 'active': self.active, 'last_image': last_image}