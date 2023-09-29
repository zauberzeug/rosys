from dataclasses import dataclass
from typing import Optional

from ..geometry import Rectangle
from .camera import Camera
from .image import ImageSize
from .image_rotation import ImageRotation


@dataclass(slots=True, kw_only=True)
class RtspCamera(Camera):
    active: bool = True
    detect: bool = False
    url: Optional[str] = None
    authorized: bool = True

    color: Optional[str] = None
    """a color code to identify the camera"""

    resolution: Optional[ImageSize] = None
    """physical resolution currently configured in the camera"""

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

    @property
    def rotation_angle(self) -> int:
        """Rotation angle in degrees."""
        return int(self.rotation)

    @rotation_angle.setter
    def rotation_angle(self, value: int) -> None:
        self.rotation = ImageRotation.from_degrees(value)

    def rotate_clockwise(self) -> None:
        """Rotate the image clockwise by 90 degrees."""
        self.rotation_angle += 90

    def rotate_counter_clockwise(self) -> None:
        """Rotate the image counter clockwise by 90 degrees."""
        self.rotation_angle -= 90
