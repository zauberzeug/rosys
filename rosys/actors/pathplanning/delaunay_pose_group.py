from dataclasses import dataclass

from ...world import Point, Pose


@dataclass(slots=True, kw_only=True)
class DelaunayPoseGroup:
    index: int
    point: Point
    neighbor_indices: list[int]
    poses: list[Pose]
