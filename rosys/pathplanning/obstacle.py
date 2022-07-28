from dataclasses import dataclass

from ..geometry import Point


@dataclass(slots=True, kw_only=True)
class Obstacle:
    id: str
    outline: list[Point]
