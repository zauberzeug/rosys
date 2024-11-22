from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from .point import Point

if TYPE_CHECKING:
    from .geo_pose import GeoPose
    from .geo_reference import GeoReference

R = 6371000


@dataclass(slots=True)
class Fixpoint:
    local_point: Point
    geo_point: GeoPoint | None = None


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
        """Calculate the distance to another point in meters using the haversine formula."""
        dlat = other.lat - self.lat
        dlon = other.lon - self.lon
        a = math.sin(dlat/2)**2 + math.cos(self.lat) * math.cos(other.lat) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    def polar(self, distance: float, direction: float) -> GeoPoint:
        """Calculate the point at a given distance and direction from this point."""
        angular_distance = distance / R
        lat2 = math.asin(math.sin(self.lat) * math.cos(angular_distance) +
                         math.cos(self.lat) * math.sin(angular_distance) * math.cos(direction))
        lon2 = self.lon + math.atan2(math.sin(direction) * math.sin(angular_distance) * math.cos(self.lat),
                                     math.cos(angular_distance) - math.sin(self.lat) * math.sin(lat2))
        return GeoPoint(lat2, lon2)

    def shifted(self, point: Point, *, reference: GeoReference | None = None) -> GeoPoint:
        """Shift by the given Cartesian coordinates (x, y) relative to the current point.
        If a reference is given, the shift is relative to the reference's direction.
        """
        distance = math.sqrt(point.x**2 + point.y**2)
        angle = math.atan2(point.y, point.x) + (reference.direction if reference else 0.0)
        return self.polar(distance, angle)

    def __str__(self) -> str:
        return f'GeoPoint(lat={math.degrees(self.lat):.6f}˚, lon={math.degrees(self.lon):.6f}˚)'

    @property
    def tuple(self) -> tuple[float, float]:
        return self.lat, self.lon

    @property
    def degree_tuple(self) -> tuple[float, float]:
        return math.degrees(self.lat), math.degrees(self.lon)
