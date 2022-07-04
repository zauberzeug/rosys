from dataclasses import dataclass

from .point import Point


@dataclass(slots=True, kw_only=True)
class Obstacle:
    id: str
    outline: list[Point]
