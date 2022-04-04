from __future__ import annotations

import abc
from typing import Any, Optional

import numpy as np
from pydantic import BaseModel, Field

from .calibration import Calibration, Extrinsics, Intrinsics
from .image import Image, ImageSize
from .rotation import Rotation


class Camera(BaseModel, abc.ABC):
    id: str
    calibration: Optional[Calibration] = None
    calibration_simulation: Optional[Calibration] = Field(None, description='only needed for simulation')
    projection: Optional[list[list[Optional[list[float]]]]] = Field(None, exclude=True)
    images: list[Image] = Field([], exclude=True)

    def __init__(self, **data: Any) -> None:
        super().__init__(**data)

    @property
    def latest_image_uri(self) -> str:
        image = self.latest_captured_image
        if image is None:
            return 'camera/placeholder'
        return f'camera/{self.id}/{image.time}'

    @property
    def captured_images(self) -> list[Image]:
        return [i for i in self.images if i.data]

    @property
    def latest_captured_image(self) -> Optional[Image]:
        images = self.captured_images
        if images:
            return images[-1]

    def set_perfect_calibration(
        self,
        x: float = 0, y: float = 0, z: float = 1,
        yaw: float = 0, tilt_x: float = 0, tilt_y: float = 0,
        image_width=800, image_height=600,
    ) -> None:
        calibration = Calibration(
            intrinsics=Camera.create_intrinsics(image_width, image_height),
            extrinsics=Extrinsics(tilt=Rotation.from_euler(tilt_x, np.pi + tilt_y, 0), yaw=yaw, translation=[x, y, z]),
        )
        self.calibration_simulation = calibration
        self.calibration = calibration.copy(deep=True)

    @staticmethod
    def create_intrinsics(image_width: int = 800, image_height: int = 600) -> Intrinsics:
        c = 570
        size = ImageSize(width=image_width, height=image_height)
        K = [[c, 0, size.width / 2], [0, c, size.height / 2], [0, 0, 1]]
        D = [0, 0, 0, 0, 0]
        rotation = Rotation.zero()
        return Intrinsics(matrix=K, distortion=D, rotation=rotation, size=size)
