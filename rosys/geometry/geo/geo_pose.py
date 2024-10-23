from __future__ import annotations

import math
from dataclasses import dataclass

from .geo_point import GeoPoint


@dataclass(slots=True)
class GeoPose:
    lat: float
    lon: float
    heading: float

    @classmethod
    def from_degrees(cls, lat: float, lon: float, heading: float) -> GeoPose:
        """Create a geo pose from latitude, longitude and heading in degrees."""
        return cls(math.radians(lat), math.radians(lon), math.radians(heading))

    @property
    def point(self) -> GeoPoint:
        """Convert the geo pose to a geo point."""
        return GeoPoint(self.lat, self.lon)

    def __str__(self) -> str:
        lat_deg = math.degrees(self.lat)
        lon_deg = math.degrees(self.lon)
        heading_deg = math.degrees(self.heading)
        return f'GeoPose(lat={lat_deg:.6f}˚, lon={lon_deg:.6f}˚, heading={heading_deg:.1f}˚)'
