from dataclasses import dataclass

from .point import Point


@dataclass(slots=True, kw_only=True)
class Polygon:
    outline: list[Point]

    def contains(self, point: Point) -> bool:
        inside = False
        for i, pi in enumerate(self.outline):
            pj = self.outline[(i + 1) % len(self.outline)]
            if ((pi.y > point.y) != (pj.y > point.y)) and (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x):
                inside = not inside
        return inside
