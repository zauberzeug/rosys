from typing import Optional

from pydantic import BaseModel

from .point import Point


class Area(BaseModel):
    id: str
    type: Optional[str]
    color: str = 'green'
    outline: list[Point]
