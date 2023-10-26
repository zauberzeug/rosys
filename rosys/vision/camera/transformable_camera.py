import abc
from typing import Optional

from ...geometry import Rectangle
from ..image import ImageSize
from ..image_rotation import ImageRotation


class TransformableCameraMixin(abc.ABC):
    crop: Optional[Rectangle]
    """region to crop on the original resolution before rotation"""

    rotation: ImageRotation
    """rotation which should be applied after grabbing and cropping"""

    _resolution: Optional[ImageSize]

    def __init__(self) -> None:
        self.crop = None
        self.rotation = ImageRotation.NONE
        self._resolution = None

    def _resolution_after_transform(self, original_resolution: ImageSize) -> Optional[ImageSize]:
        width = int(self.crop.width) if self.crop else original_resolution.width
        height = int(self.crop.height) if self.crop else original_resolution.height
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
