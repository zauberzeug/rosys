from dataclasses import dataclass

from .spline import Spline


@dataclass(slots=True, kw_only=True)
class PathSegment:
    spline: Spline
    backward: bool = False
