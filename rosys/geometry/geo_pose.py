from __future__ import annotations

import math
from dataclasses import dataclass

from .geo_point import GeoPoint
from .geo_reference import current_geo_reference
from .pose import Pose


@dataclass(slots=True)
class GeoPose:
    lat: float
    lon: float
    heading: float

    @classmethod
    def from_degrees(cls, lat: float, lon: float, heading: float) -> GeoPose:
        """Create a geo pose from latitude, longitude and heading in degrees."""
        return cls(math.radians(lat), math.radians(lon), math.radians(heading))

    @classmethod
    def from_pose(cls, pose: Pose) -> GeoPose:
        """Create a geo pose from a pose."""
        geo_point1 = current_geo_reference.point_to_geo(pose)
        geo_point2 = current_geo_reference.point_to_geo(pose.transform_pose(Pose(x=1)))
        return cls(geo_point1.lat, geo_point1.lon, geo_point1.direction(geo_point2))

    @property
    def point(self) -> GeoPoint:
        """Convert the geo pose to a geo point."""
        return GeoPoint(self.lat, self.lon)

    def cartesian(self) -> Pose:
        """Convert the geo pose to a local pose."""
        point1 = current_geo_reference.point_to_local(self)
        point2 = current_geo_reference.point_to_local(self.point.polar(1, self.heading))
        return Pose(x=point1.x, y=point1.y, yaw=point1.direction(point2))

    def __str__(self) -> str:
        lat_deg, lon_deg, heading_deg = self.degree_tuple
        return f'GeoPose(lat={lat_deg:.6f}˚, lon={lon_deg:.6f}˚, heading={heading_deg:.1f}˚)'

    @property
    def degree_tuple(self) -> tuple[float, float, float]:
        return math.degrees(self.lat), math.degrees(self.lon), math.degrees(self.heading)

    @property
    def tuple(self) -> tuple[float, float, float]:
        return self.lat, self.lon, self.heading
