from __future__ import annotations

import abc
from typing import Optional
from uuid import uuid4

import numpy as np
from pydantic import BaseModel, Field

from .calibration import Calibration, Extrinsics, Intrinsics
from .image import Image, ImageSize
from .rotation import Rotation

placeholder = Image.create_placeholder('no image')


class Camera(BaseModel, abc.ABC):
    id: str
    calibration: Optional[Calibration] = None
    calibration_simulation: Optional[Calibration] = Field(None, description='only needed for simulation')
    projection: Optional[list[list[Optional[list[float]]]]] = Field(None, exclude=True)
    images: list[Image] = Field([placeholder], exclude=True)

    @property
    def latest_image_uri(self):
        if not self.images:
            return 'camera/placeholder'
        return f'camera/{self.id}/{self.latest_captured_image[-1].time}'

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
    ) -> None:
        calibration = Calibration(
            intrinsics=Camera.create_intrinsics(),
            extrinsics=Extrinsics(tilt=Rotation.from_euler(tilt_x, np.pi + tilt_y, 0), yaw=yaw, translation=[x, y, z]),
        )
        self.calibration_simulation = calibration
        self.calibration = calibration.copy(deep=True)

    @staticmethod
    def create_intrinsics() -> Intrinsics:
        c = 570
        size = ImageSize(width=800, height=600)
        K = [[c, 0, size.width / 2], [0, c, size.height / 2], [0, 0, 1]]
        D = [0, 0, 0, 0, 0]
        rotation = Rotation.zero()
        return Intrinsics(matrix=K, distortion=D, rotation=rotation, size=size)
