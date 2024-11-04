from ... import persistence
from ...geometry import Rectangle
from ..image import ImageSize
from ..image_rotation import ImageRotation
from .camera import Camera


class TransformableCamera(Camera):

    def __init__(self, crop: Rectangle | dict | None = None, rotation: ImageRotation | int = ImageRotation.NONE, **kwargs) -> None:
        super().__init__(**kwargs)
        self.crop: Rectangle | None = None
        """region to crop on the original resolution before rotation"""
        if isinstance(crop, Rectangle):
            self.crop = crop
        elif isinstance(crop, dict):
            self.crop = persistence.from_dict(Rectangle, crop)

        self.rotation: ImageRotation = rotation if isinstance(
            rotation, ImageRotation) else ImageRotation.from_degrees(rotation)
        """rotation which should be applied after grabbing and cropping"""

        self._resolution: ImageSize | None = None

    def to_dict(self) -> dict:
        return super().to_dict() | {
            'crop': persistence.to_dict(self.crop),
            'rotation': self.rotation.value,
        }

    def _resolution_after_transform(self, original_resolution: ImageSize) -> ImageSize:
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
