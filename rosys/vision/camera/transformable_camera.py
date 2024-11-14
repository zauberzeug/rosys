from ... import persistence
from ...geometry import Rectangle
from ..image_rotation import ImageRotation
from .camera import Camera


class TransformableCamera(Camera):

    def __init__(self, *, crop: Rectangle | dict | None = None, rotation: ImageRotation | int = ImageRotation.NONE, **kwargs) -> None:
        """
        A camera for which the image can be cropped and rotated.

        This mixin does not implement the actual cropping and rotation.
        It only provides the infrastructure for storing the crop and rotation parameters.

        :param crop: region to crop on the original resolution before rotation
        :param rotation: clockwise rotation which should be applied after cropping
        """
        super().__init__(**kwargs)

        self.crop: Rectangle | None = None
        if isinstance(crop, Rectangle):
            self.crop = crop
        elif isinstance(crop, dict):
            self.crop = persistence.from_dict(Rectangle, crop)

        self.rotation: ImageRotation = rotation if isinstance(
            rotation, ImageRotation) else ImageRotation.from_degrees(rotation)

    def to_dict(self) -> dict:
        return super().to_dict() | {
            'crop': persistence.to_dict(self.crop),
            'rotation': self.rotation.value,
        }

    @property
    def rotation_angle(self) -> int:
        """Clockwise rotation angle in degrees."""
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
