from __future__ import annotations

from dataclasses import dataclass

from .point import Point


@dataclass(slots=True, kw_only=True)
class Rectangle:
    x: float
    y: float
    width: float
    height: float

    def __post_init__(self) -> None:
        if self.width < 0 or self.height < 0:
            raise ValueError(f'Rectangle dimensions must be non-negative, got width={self.width}, height={self.height}')

    @property
    def tuple(self) -> tuple[float, float, float, float]:
        return (self.x, self.y, self.width, self.height)

    def contains(self, point: Point) -> bool:
        return (self.x <= point.x <= self.x + self.width and
                self.y <= point.y <= self.y + self.height)

    def __str__(self) -> str:
        return f'Rectangle({round(self.x, 2)}, {round(self.y, 2)}, {round(self.width, 2)}, {round(self.height, 2)})'
