from enum import Enum
from typing import Self


class ImageRotation(str, Enum):
    NONE: int = 0
    RIGHT: int = 90
    UPSIDE_DOWN: int = 180
    LEFT: int = 270

    @classmethod
    def from_degrees(cls, degrees: int) -> Self:
        for rotation in cls:
            if int(rotation.value) == degrees % 360:
                return rotation
        raise ValueError(f'invalid rotation angle: {degrees}')
