from pydantic import BaseModel
from .spline import Spline


class PathSegment(BaseModel):
    spline: Spline
    backward: bool = False
