from __future__ import annotations

import math
from dataclasses import dataclass
from typing import ClassVar

from .point import Point
from .pose import Pose

RADIUS = 6371000.0
ZERO_POINT = Point(x=0, y=0)


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
        """Create a geo point from a local point in the current geo reference."""
        assert GeoReference.current is not None
        return GeoReference.current.point_to_geo(point)

    def direction(self, other: GeoPoint | GeoPose) -> float:
        """Calculate the direction to another geo point."""
        return math.atan2(math.sin(other.lon - self.lon) * math.cos(other.lat),
                          math.cos(self.lat) * math.sin(other.lat) -
                          math.sin(self.lat) * math.cos(other.lat) * math.cos(other.lon - self.lon))

    def distance(self, other: GeoPoint | GeoPose) -> float:
        """Calculate the distance to another geo point in meters using the Haversine formula."""
        d_lat = other.lat - self.lat
        d_lon = other.lon - self.lon
        a = math.sin(d_lat / 2)**2 + math.cos(self.lat) * math.cos(other.lat) * math.sin(d_lon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return RADIUS * c

    def polar(self, distance: float, direction: float) -> GeoPoint:
        """Calculate the geo point at a given distance and direction from this geo point."""
        angular_distance = distance / RADIUS
        lat2 = math.asin(math.sin(self.lat) * math.cos(angular_distance) +
                         math.cos(self.lat) * math.sin(angular_distance) * math.cos(direction))
        lon2 = self.lon + math.atan2(math.sin(direction) * math.sin(angular_distance) * math.cos(self.lat),
                                     math.cos(angular_distance) - math.sin(self.lat) * math.sin(lat2))
        return GeoPoint(lat2, lon2)

    def shift_by(self, *, x: float = 0.0, y: float = 0.0) -> GeoPoint:
        """Shift the geo point by Cartesian coordinates in meters relative to the current geo reference.

        If no current geo reference is set, x will be applied in North direction and y in West direction.
        """
        distance = math.sqrt(x**2 + y**2)
        angle = math.atan2(-y, x) + GeoReference.current.direction if GeoReference.current is not None else 0
        return self.polar(distance, angle)

    def to_local(self) -> Point:
        """Transform to local Cartesian coordinates relative to the current geo reference."""
        assert GeoReference.current is not None
        return GeoReference.current.point_to_local(self)

    def __str__(self) -> str:
        lat_deg, lon_deg = self.degree_tuple
        return f'GeoPoint(lat={lat_deg:.6f}˚, lon={lon_deg:.6f}˚)'

    @property
    def degree_tuple(self) -> tuple[float, float]:
        """Latitude and longitude in degrees."""
        return math.degrees(self.lat), math.degrees(self.lon)

    @property
    def tuple(self) -> tuple[float, float]:
        """Latitude and longitude in radians."""
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
        """Create a geo pose from a pose in the current geo reference."""
        assert GeoReference.current is not None
        geo_point1 = GeoReference.current.point_to_geo(pose)
        geo_point2 = GeoReference.current.point_to_geo(pose.transform_pose(Pose(x=1)))
        return cls(geo_point1.lat, geo_point1.lon, geo_point1.direction(geo_point2))

    @property
    def point(self) -> GeoPoint:
        """The geo point of the geo pose."""
        return GeoPoint(self.lat, self.lon)

    def relative_shift_by(self, *, x: float = 0.0, y: float = 0.0) -> GeoPose:
        """Shift the geo pose by Cartesian coordinates in meters relative to its current heading."""
        distance = math.sqrt(x**2 + y**2)
        angle = math.atan2(-y, x) + self.heading
        shifted_point = self.point.polar(distance, angle)
        return GeoPose(shifted_point.lat, shifted_point.lon, self.heading)

    def to_local(self) -> Pose:
        """Transform to local Cartesian coordinates relative to the current geo reference."""
        assert GeoReference.current is not None
        point1 = GeoReference.current.point_to_local(self)
        point2 = GeoReference.current.point_to_local(self.point.polar(1, self.heading))
        return Pose(x=point1.x, y=point1.y, yaw=point1.direction(point2))

    def __str__(self) -> str:
        lat_deg, lon_deg, heading_deg = self.degree_tuple
        return f'GeoPose(lat={lat_deg:.6f}˚, lon={lon_deg:.6f}˚, heading={heading_deg:.1f}˚)'

    @property
    def degree_tuple(self) -> tuple[float, float, float]:
        """Latitude, longitude, and heading in degrees."""
        return math.degrees(self.lat), math.degrees(self.lon), math.degrees(self.heading)

    @property
    def tuple(self) -> tuple[float, float, float]:
        """Latitude, longitude, and heading in radians."""
        return self.lat, self.lon, self.heading


@dataclass(slots=True)
class Fixpoint:
    local_point: Point
    geo_point: GeoPoint | None = None


@dataclass(slots=True)
class GeoReference:
    current: ClassVar[GeoReference | None] = None
    origin: GeoPoint
    direction: float = 0.0  # direction of the local x-axis in the global geo system (0 = north, pi/2 = east)

    @classmethod
    def update_current(cls, new_reference: GeoReference) -> None:
        """Update the current geo reference to the given reference."""
        if cls.current is None:
            cls.current = new_reference
        else:
            cls.current.origin = new_reference.origin
            cls.current.direction = new_reference.direction

    @classmethod
    def from_two_fixpoints(cls, A: Fixpoint, B: Fixpoint) -> GeoReference:
        """Create a geo reference from two fixpoints.

        :param A: The first fixpoint.
        :param B: The second fixpoint.
        :raises: AssertionError: If either fixpoint does not have a geo_point.
        """
        assert A.geo_point is not None, 'Fixpoint "A" must have a geo_point'
        assert B.geo_point is not None, 'Fixpoint "B" must have a geo_point'
        x_direction = A.geo_point.direction(B.geo_point) + A.local_point.direction(B.local_point)
        origin = A.geo_point \
            .polar(A.local_point.x, x_direction + math.radians(180)) \
            .polar(A.local_point.y, x_direction + math.radians(90))
        return cls(origin, x_direction)

    def point_to_geo(self, point: Point | Pose) -> GeoPoint:
        """Convert a local point to a global point."""
        return self.origin.polar(ZERO_POINT.distance(point), self.direction - ZERO_POINT.direction(point))

    def pose_to_geo(self, pose: Pose) -> GeoPose:
        """Convert a local pose to a global geo pose."""
        geo_point1 = self.point_to_geo(pose)
        geo_point2 = self.point_to_geo(pose.transform_pose(Pose(x=1)))
        return GeoPose(geo_point1.lat, geo_point1.lon, geo_point1.direction(geo_point2))

    def point_to_local(self, geo_point: GeoPoint | GeoPose) -> Point:
        """Convert a global point to a local point."""
        return ZERO_POINT.polar(self.origin.distance(geo_point), self.direction - self.origin.direction(geo_point))

    def pose_to_local(self, geo_pose: GeoPose) -> Pose:
        """Convert a global geo pose to a local pose."""
        point1 = self.point_to_local(geo_pose)
        point2 = self.point_to_local(geo_pose.point.polar(1, geo_pose.heading))
        return Pose(x=point1.x, y=point1.y, yaw=point1.direction(point2))

    @property
    def degree_tuple(self) -> tuple[float, float, float]:
        """Latitude, longitude, and direction of the x-axis in degrees."""
        return (*self.origin.degree_tuple, math.degrees(self.direction))

    @property
    def tuple(self) -> tuple[float, float, float]:
        """Latitude, longitude, and direction of the x-axis in radians."""
        return (*self.origin.tuple, self.direction)

    def __str__(self) -> str:
        lat_deg, lon_deg, direction_deg = self.degree_tuple
        return f'GeoReference(lat={lat_deg:.6f}˚, lon={lon_deg:.6f}˚, heading={direction_deg:.1f}˚)'
