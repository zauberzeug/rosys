from dataclasses import dataclass
from itertools import combinations, pairwise

from typing_extensions import Self

from ..geometry import LineSegment, Point, Polygon


@dataclass(slots=True, kw_only=True)
class Area(Polygon):
    id: str
    type: str | None = None
    color: str = 'green'
    closed: bool = True

    def close(self) -> Self:
        self.closed = True
        return self

    def would_cause_self_intersection(self, point: Point, new_index: int | None = None) -> bool:
        new_outline = self.outline[:]
        if new_index is None:
            new_outline.append(point)
        else:
            new_outline.insert(new_index, point)
        line_segments = [LineSegment(point1=p1, point2=p2) for p1, p2 in pairwise(new_outline + new_outline[:1])]
        if not self.closed:
            line_segments.pop()
        for segment1, segment2 in combinations(line_segments, 2):
            if id(point) not in {id(segment1.point1), id(segment1.point2), id(segment2.point1), id(segment2.point2)}:
                continue  # segments are not affected by the new point
            if {id(segment1.point1), id(segment1.point2)}.intersection({id(segment2.point1), id(segment2.point2)}):
                continue  # segments are connected by a shared point
            if segment1.intersect(segment2):
                return True
        return False
