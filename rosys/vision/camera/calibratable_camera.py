from typing import Optional, Self

import numpy as np

from ...geometry import Rotation
from ..calibration import Calibration, Extrinsics, Intrinsics
from ..image import ImageSize
from .camera import Camera


class CalibratableCameraMixin(Camera):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.calibration: Optional[Calibration] = None
        self.focal_length: Optional[float] = None

    @property
    def is_calibrated(self) -> bool:
        return self.calibration is not None

    @classmethod
    def create_calibrated(cls, *,
                          width: int = 800, height: int = 600,
                          x: float = 0.0, y: float = 0.0, z: float = 1.0,
                          roll: float = np.pi, pitch: float = 0.0, yaw: float = 0.0,
                          **kwargs) -> Self:
        camera = cls(**kwargs)
        camera.set_perfect_calibration(width=width, height=height, x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)
        return camera

    def set_perfect_calibration(self, *,
                                width=800, height=600,
                                x: float = 0.0, y: float = 0.0, z: float = 1.0,
                                roll: float = np.pi, pitch: float = 0.0, yaw: float = 0.0) -> None:
        self.calibration = Calibration(
            intrinsics=self.create_intrinsics(width, height),
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
