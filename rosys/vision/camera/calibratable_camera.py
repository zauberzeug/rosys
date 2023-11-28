from typing import Optional, Self

import numpy as np

from ...geometry import Rotation
from ..calibration import Calibration, Extrinsics, Intrinsics
from .camera import Camera


class CalibratableCamera(Camera):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.calibration: Optional[Calibration] = None

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
            intrinsics=Intrinsics.create_default(width, height),
            extrinsics=Extrinsics(rotation=Rotation.from_euler(roll, pitch, yaw), translation=[x, y, z]),
        )
