from __future__ import annotations

from dataclasses import dataclass

from .point import Point


@dataclass(slots=True, kw_only=True)
class Rectangle:
    x: float
    y: float
    width: float
    height: float

    @property
    def tuple(self) -> tuple[float, float, float, float]:
        return (self.x, self.y, self.width, self.height)

    def contains(self, point: Point) -> bool:
        return (self.x <= point.x <= self.x + self.width and
                self.y <= point.y <= self.y + self.height)

    def __str__(self) -> str:
        return f'Rectangle({round(self.x, 2)}, {round(self.y, 2)}, {round(self.width, 2)}, {round(self.height, 2)})'
