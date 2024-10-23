from dataclasses import dataclass

import rosys

from .geo_point import GeoPoint


@dataclass(slots=True)
class Fixpoint:
    local_point: rosys.geometry.Point
    geo_point: GeoPoint | None = None
