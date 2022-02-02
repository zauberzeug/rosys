from pydantic import BaseModel
from .point import Point


class Area(BaseModel):
    id: str
    outline: list[Point]
