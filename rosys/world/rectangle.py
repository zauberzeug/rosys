from __future__ import annotations

from pydantic import BaseModel

from .point import Point


class Rectangle(BaseModel):
    x: float
    y: float
    width: float
    height: float

    @property
    def tuple(self) -> tuple[float, float, float, float]:
        return (self.x, self.y, self.w, self.h)

    def contains(self, point: Point) -> bool:
        return (self.x <= point.x <= self.x + self.width and
                self.y <= point.y <= self.y + self.height)

    def __str__(self) -> str:
        return f'Rectangle({round(self.x, 2)}, {round(self.y, 2)}, {round(self.w, 2)}, {round(self.h, 2)})'
