from dataclasses import dataclass
from typing import Optional, Self

from ..geometry import Polygon


@dataclass(slots=True, kw_only=True)
class Area(Polygon):
    id: str
    type: Optional[str] = None
    color: str = 'green'
    closed: bool = True

    def close(self) -> Self:
        self.closed = True
        return self
