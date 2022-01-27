from __future__ import annotations
from typing import List, Optional
from uuid import uuid4
from pydantic import BaseModel, Field
import numpy as np
from .image import Image, ImageSize
from .rotation import Rotation
from .calibration import Calibration, Extrinsics, Intrinsics


no_img_placeholder = Image.create_placeholder('no image')


class Camera(BaseModel):
    id: str
    calibration: Optional[Calibration] = None
    size: Optional[ImageSize] = None
    images: List[Image] = Field([no_img_placeholder], exclude=True)

    @property
    def latest_image_uri(self):
        return f'camera/{self.id}/{self.images[-1].time}'

    @staticmethod
    def create_perfect_camera(
        x: float = 0, y: float = 0, z: float = 1,
        yaw: float = 0, tilt_x: float = 0, tilt_y: float = 0,
        id: str = None,
    ) -> Camera:
        calibration = Calibration(
            intrinsics=Camera.create_intrinsics(),
            extrinsics=Extrinsics(tilt=Rotation.from_euler(tilt_x, np.pi + tilt_y, 0), yaw=yaw, translation=[x, y, z]),
        )
        return Camera(
            id=id or str(uuid4()),
            calibration_simulation=calibration,
            calibration=calibration.copy(deep=True),
            size=ImageSize(calibration.intrinsics.size.width, calibration.intrinsics.size.height),
        )

    @staticmethod
    def create_intrinsics() -> Intrinsics:
        c = 570
        size = ImageSize(width=1600, height=1200)
        K = [[c, 0, size.width / 2], [0, c, size.height / 2], [0, 0, 1]]
        D = [0, 0, 0, 0, 0]
        rotation = Rotation.zero()
        return Intrinsics(matrix=K, distortion=D, rotation=rotation, size=size)
