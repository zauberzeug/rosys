from typing import Optional

import numpy as np
from typing_extensions import Self

from ...geometry import Point3d, Pose3d, Rotation
from ..calibration import Calibration, Intrinsics
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
            extrinsics=Pose3d(rotation=Rotation.from_euler(roll, pitch, yaw),
                              translation=Point3d.from_tuple([x, y, z])),
        )
