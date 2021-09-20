from pydantic import BaseModel
from .point import Point


class Obstacle(BaseModel):
    id: str
    outline: list[Point]
