from enum import Enum

from typing_extensions import Self


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
        raise ValueError(f'invalid rotation angle: {degrees}')
