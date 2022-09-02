from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from rosys import persistence

from ..geometry import Rotation
from .calibration import Calibration, Extrinsics, Intrinsics
from .image import Image, ImageSize


@dataclass(slots=True, kw_only=True)
class Camera(abc.ABC):
    id: str
    calibration: Optional[Calibration] = None
    images: list[Image] = field(default_factory=list, metadata=persistence.exclude)

    @property
    def captured_images(self) -> list[Image]:
        return [i for i in self.images if i.data]

    @property
    def latest_captured_image(self) -> Optional[Image]:
        images = self.captured_images
        if images:
            return images[-1]

    def get_recent_images(self, current_time: float, timespan: float = 10.0) -> list[Image]:
        return [i for i in self.captured_images if i.time > current_time - timespan]

    def set_perfect_calibration(
        self,
        *,
        width=800, height=600,
        x: float = 0.0, y: float = 0.0, z: float = 1.0,
        roll: float = np.pi, pitch: float = 0.0, yaw: float = 0.0,
    ) -> None:
        self.calibration = Calibration(
            intrinsics=Camera.create_intrinsics(width, height),
            extrinsics=Extrinsics(rotation=Rotation.from_euler(roll, pitch, yaw), translation=[x, y, z]),
        )

    @staticmethod
    def create_intrinsics(width: int = 800, height: int = 600) -> Intrinsics:
        c = 570
        size = ImageSize(width=width, height=height)
        K = [[c, 0, size.width / 2], [0, c, size.height / 2], [0, 0, 1]]
        D = [0, 0, 0, 0, 0]
        rotation = Rotation.zero()
        return Intrinsics(matrix=K, distortion=D, rotation=rotation, size=size)
