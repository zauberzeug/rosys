from __future__ import annotations

import abc
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .. import persistence, rosys
from ..geometry import Rectangle, Rotation
from .calibration import Calibration, Extrinsics, Intrinsics
from .image import Image, ImageSize
from .image_rotation import ImageRotation


class ExposureCameraMixin(abc.ABC):
    auto_exposure: Optional[bool] = True
    """toggles auto exposure"""

    exposure: Optional[float] = None
    """manual exposure of the camera (between 0-1); set auto_exposure to False for this value to take effect"""

    @abc.abstractmethod
    async def set_exposure(self) -> None:
        pass


class TransformCameraMixin(abc.ABC):
    crop: Optional[Rectangle] = None
    """region to crop on the original resolution before rotation"""

    rotation: ImageRotation = ImageRotation.NONE
    """rotation which should be applied after grabbing and cropping"""

    resolution: Optional[ImageSize] = None

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


class CalibratedCameraMixin(abc.ABC):
    calibration: Optional[Calibration] = None
    focal_length: Optional[float] = None

    @property
    def is_calibrated(self) -> bool:
        return self

    def set_perfect_calibration(
        self,
        *,
        width=800, height=600,
        x: float = 0.0, y: float = 0.0, z: float = 1.0,
        roll: float = np.pi, pitch: float = 0.0, yaw: float = 0.0,
    ) -> None:
        self.calibration = Calibration(
            intrinsics=CalibratedCameraMixin.create_intrinsics(width, height),
            extrinsics=Extrinsics(rotation=Rotation.from_euler(roll, pitch, yaw), translation=[x, y, z]),
        )

    @staticmethod
    def create_intrinsics(width: int = 800, height: int = 600) -> Intrinsics:
        c = 570
        size = ImageSize(width=width, height=height)
        K: list[list[float]] = [[c, 0, size.width / 2], [0, c, size.height / 2], [0, 0, 1]]
        D: list[float] = [0] * 5
        rotation = Rotation.zero()
        return Intrinsics(matrix=K, distortion=D, rotation=rotation, size=size)


@dataclass(slots=True, kw_only=True)
class Camera(abc.ABC):
    MAX_IMAGES = 256

    id: str
    images: deque[Image] = field(default_factory=lambda: deque(maxlen=Camera.MAX_IMAGES), metadata=persistence.exclude)
    name: Optional[str] = None

    activate_after_init: bool = True

    fps: Optional[int] = None
    """current frames per second (read only)"""

    _resolution: Optional[ImageSize] = None
    """physical resolution of the camera which should be used; camera may go into error state with wrong values"""

    _activation_lock: asyncio.Lock = field(default_factory=asyncio.Lock, metadata=persistence.exclude)
    activating: bool = False

    def __post_init__(self) -> None:
        if self.name is None:
            self.name = self.id
        if self.activate_after_init:
            # start a new task to activate the camera
            rosys.on_startup(self.activate)

    @property
    def image_resolution(self) -> Optional[ImageSize]:
        return self._resolution

    @property
    def is_connected(self) -> bool:
        """To be interpreted as "ready to capture images"."""
        return False

    async def activate(self) -> None:
        # NOTE: due to a race condition, this method may be called multiple times even after any check for is_connected
        #       Make sure it is idempotent!
        pass

    async def deactivate(self) -> None:
        pass

    @property
    def captured_images(self) -> list[Image]:
        return [i for i in self.images if i.data]

    @property
    def latest_captured_image(self) -> Optional[Image]:
        images = self.captured_images
        return images[-1] if images else None

    def get_recent_images(self, current_time: float, timespan: float = 10.0) -> list[Image]:
        return [i for i in self.captured_images if i.time > current_time - timespan]
