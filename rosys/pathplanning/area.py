from dataclasses import dataclass
from typing import Optional

from ..geometry import Polygon


@dataclass(slots=True, kw_only=True)
class Area(Polygon):
    id: str
    type: Optional[str] = None
    color: str = 'green'
    closed: bool = True
