from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .geo_pose import GeoPose

R = 6371000


@dataclass(slots=True)
class GeoPoint:
    lat: float
    lon: float

    @classmethod
    def from_degrees(cls, lat: float, lon: float) -> GeoPoint:
        """Create a geo point from latitude and longitude in degrees."""
        return cls(math.radians(lat), math.radians(lon))

    def direction(self, other: GeoPoint | GeoPose) -> float:
        """Calculate the direction to another point in degrees."""
        return math.atan2(math.sin(other.lon - self.lon) * math.cos(other.lat),
                          math.cos(self.lat) * math.sin(other.lat) -
                          math.sin(self.lat) * math.cos(other.lat) * math.cos(other.lon - self.lon))

    def distance(self, other: GeoPoint | GeoPose) -> float:
        """Calculate the distance to another point in meters."""
        return R * math.acos(math.sin(self.lat) * math.sin(other.lat) +
                             math.cos(self.lat) * math.cos(other.lat) * math.cos(other.lon - self.lon))

    def polar(self, distance: float, direction: float) -> GeoPoint:
        """Calculate the point at a given distance and direction from this point."""
        angular_distance = distance / R
        lat2 = math.asin(math.sin(self.lat) * math.cos(angular_distance) +
                         math.cos(self.lat) * math.sin(angular_distance) * math.cos(direction))
        lon2 = self.lon + math.atan2(math.sin(direction) * math.sin(angular_distance) * math.cos(self.lat),
                                     math.cos(angular_distance) - math.sin(self.lat) * math.sin(lat2))
        return GeoPoint(lat2, lon2)

    def __str__(self) -> str:
        return f'GeoPoint(lat={math.degrees(self.lat):.6f}˚, lon={math.degrees(self.lon):.6f}˚)'
