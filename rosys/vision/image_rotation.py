from enum import Enum
from typing import Self


class ImageRotation(int, Enum):
    NONE = 0
    RIGHT = 90
    UPSIDE_DOWN = 180
    LEFT = 270

    @classmethod
    def from_degrees(cls, degrees: int) -> Self:
        for rotation in cls:
            if int(rotation.value) == degrees % 360:
                return rotation
        raise ValueError(f'Invalid rotation angle: {degrees}. mod(angle, 360) must be one of 0, 90, 180, 270')
