from __future__ import annotations

import math
from dataclasses import dataclass

from .. import helpers
from .point import Point
from .pose import Pose

R = 6371000
ZERO_POINT = Point(x=0, y=0)
current_geo_reference: GeoReference


@dataclass(slots=True)
class GeoPoint:
    lat: float
    lon: float

    @classmethod
    def from_degrees(cls, lat: float, lon: float) -> GeoPoint:
        """Create a geo point from latitude and longitude in degrees."""
        return cls(math.radians(lat), math.radians(lon))

    @classmethod
    def from_point(cls, point: Point) -> GeoPoint:
        """Create a geo point from a local point."""
        return current_geo_reference.point_to_geo(point)

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

    def shifted(self, point: Point) -> GeoPoint:
        """Shift by the given Cartesian coordinates (x, y) relative to the current point.
        (x, y) are in the local coordinate frame where x is the direction of the reference in a right hand frame.
        """
        distance = math.sqrt(point.x**2 + point.y**2)
        reference_direction = current_geo_reference.direction if current_geo_reference.is_set else 0
        angle = math.atan2(point.y, point.x) + helpers.angle(math.radians(180), reference_direction)
        return self.polar(distance, angle)

    def cartesian(self) -> Point:
        """Transform to local cartesian coordinates relative to current reference."""
        return current_geo_reference.point_to_local(self)

    def __str__(self) -> str:
        lat_deg, lon_deg = self.degree_tuple
        return f'GeoPoint(lat={lat_deg:.6f}˚, lon={lon_deg:.6f}˚)'

    @property
    def degree_tuple(self) -> tuple[float, float]:
        return math.degrees(self.lat), math.degrees(self.lon)

    @property
    def tuple(self) -> tuple[float, float]:
        return self.lat, self.lon


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


@dataclass(slots=True)
class Fixpoint:
    local_point: Point
    geo_point: GeoPoint | None = None


class MissingGeoReferenceError(Exception):
    """Raised when a geo reference is required but not set."""

    def __str__(self) -> str:
        return 'Geo reference is not set. See rosys.geometry.geo_reference.current_geo_reference'


@dataclass(slots=True)
class GeoReference:
    origin: GeoPoint | None = None
    direction: float = 0.0  # direction of the local x-axis in the global geo system (0 = north, pi/2 = east)

    @property
    def is_set(self) -> bool:
        return self.origin is not None

    @classmethod
    def from_two_fixpoints(cls, A: Fixpoint, B: Fixpoint) -> GeoReference:
        """Create a geo reference from two fixpoints."""
        assert A.geo_point is not None, 'Fixpoint "A" must have a geo_point'
        assert B.geo_point is not None, 'Fixpoint "B" must have a geo_point'
        x_direction = A.geo_point.direction(B.geo_point) + A.local_point.direction(B.local_point)
        origin = A.geo_point \
            .polar(A.local_point.x, x_direction + math.radians(180)) \
            .polar(A.local_point.y, x_direction + math.radians(90))
        return cls(origin, x_direction)

    # TODO: provide GeoReference or GeoPoint and Heading?
    def update(self, new_reference: GeoReference) -> None:
        """Update the current reference to the given reference."""
        self.origin = new_reference.origin
        self.direction = new_reference.direction

    def point_to_geo(self, point: Point | Pose) -> GeoPoint:
        """Convert a local point to a global point."""
        if not self.is_set:
            raise MissingGeoReferenceError
        assert self.origin is not None
        return self.origin.polar(ZERO_POINT.distance(point), self.direction - ZERO_POINT.direction(point))

    def pose_to_geo(self, pose: Pose) -> GeoPose:
        """Convert a local pose to a global geo pose."""
        geo_point1 = self.point_to_geo(pose)
        geo_point2 = self.point_to_geo(pose.transform_pose(Pose(x=1)))
        return GeoPose(geo_point1.lat, geo_point1.lon, geo_point1.direction(geo_point2))

    def point_to_local(self, geo_point: GeoPoint | GeoPose) -> Point:
        """Convert a global point to a local point."""
        if not self.is_set:
            raise MissingGeoReferenceError
        assert self.origin is not None
        return ZERO_POINT.polar(self.origin.distance(geo_point), self.direction - self.origin.direction(geo_point))

    def pose_to_local(self, geo_pose: GeoPose) -> Pose:
        """Convert a global geo pose to a local pose."""
        point1 = self.point_to_local(geo_pose)
        point2 = self.point_to_local(geo_pose.point.polar(1, geo_pose.heading))
        return Pose(x=point1.x, y=point1.y, yaw=point1.direction(point2))

    @property
    def degree_tuple(self) -> tuple[float, float, float]:
        """Latitude, longitude, and direction (in global geo system, in degrees)."""
        if not self.is_set:
            raise MissingGeoReferenceError
        assert self.origin is not None
        return (*self.origin.degree_tuple, math.degrees(self.direction))

    @property
    def tuple(self) -> tuple[float, float, float]:
        """Latitude, longitude, and direction (in global geo system)."""
        if not self.is_set:
            raise MissingGeoReferenceError
        assert self.origin is not None
        return (*self.origin.tuple, self.direction)

    def __str__(self) -> str:
        lat_deg, lon_deg, direction_deg = self.degree_tuple
        return f'GeoReference(lat={lat_deg:.6f}˚, lon={lon_deg:.6f}˚, heading={direction_deg:.1f}˚)'


# TODO: naming
current_geo_reference = GeoReference()
