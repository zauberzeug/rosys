import numpy as np
from typing_extensions import Self

from ...geometry import Rotation
from ..calibration import Calibration, Extrinsics, Intrinsics
from .camera import Camera


class CalibratableCamera(Camera):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.calibration: Calibration | None = None

    @property
    def is_calibrated(self) -> bool:
        return self.calibration is not None

    @classmethod
    def create_calibrated(cls, *,
                          width: int = 800, height: int = 600, focal_length: float = 570,
                          x: float = 0.0, y: float = 0.0, z: float = 1.0,
                          roll: float = np.pi, pitch: float = 0.0, yaw: float = 0.0,
                          **kwargs) -> Self:
        camera = cls(**kwargs)
        camera.set_perfect_calibration(width=width, height=height, focal_length=focal_length,
                                       x=x, y=y, z=z,
                                       roll=roll, pitch=pitch, yaw=yaw)
        return camera

    def set_perfect_calibration(self, *,
                                width: int = 800, height: int = 600, focal_length: float = 570,
                                x: float = 0.0, y: float = 0.0, z: float = 1.0,
                                roll: float = np.pi, pitch: float = 0.0, yaw: float = 0.0,
                                ) -> None:
        self.calibration = Calibration(
            intrinsics=Intrinsics.create_default(width, height, focal_length=focal_length),
            extrinsics=Extrinsics(rotation=Rotation.from_euler(roll, pitch, yaw), translation=[x, y, z]),
        )
