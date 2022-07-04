from dataclasses import dataclass
from typing import Optional

from .point import Point


@dataclass(slots=True, kw_only=True)
class Area:
    id: str
    type: Optional[str] = None
    color: str = 'green'
    outline: list[Point]
