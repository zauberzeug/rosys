from __future__ import annotations

import math
from dataclasses import dataclass

import rosys

from .fixpoint import Fixpoint
from .geo_point import GeoPoint
from .geo_pose import GeoPose

ZERO_POINT = rosys.geometry.Point(x=0, y=0)


@dataclass(slots=True)
class GeoReference:
    origin: GeoPoint
    direction: float  # direction of the local x-axis in the global geo system (0 = north, pi/2 = east)

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

    def pose_to_geo(self, pose: rosys.geometry.Pose) -> GeoPose:
        """Convert a local pose to a global geo pose."""
        geo_point1 = self.point_to_geo(pose)
        geo_point2 = self.point_to_geo(pose.transform_pose(rosys.geometry.Pose(x=1)))
        return GeoPose(geo_point1.lat, geo_point1.lon, geo_point1.direction(geo_point2))

    def point_to_geo(self, point: rosys.geometry.Point | rosys.geometry.Pose) -> GeoPoint:
        """Convert a local point to a global point (latitude, longitude; all in degrees)."""
        return self.origin.polar(ZERO_POINT.distance(point), self.direction - ZERO_POINT.direction(point))

    def pose_to_local(self, geo_pose: GeoPose) -> rosys.geometry.Pose:
        """Convert a global geo pose to a local pose."""
        point1 = self.point_to_local(geo_pose)
        point2 = self.point_to_local(geo_pose.point.polar(1, geo_pose.heading))
        return rosys.geometry.Pose(x=point1.x, y=point1.y, yaw=point1.direction(point2))

    def point_to_local(self, geo_point: GeoPoint | GeoPose) -> rosys.geometry.Point:
        """Convert a global point to a local point."""
        return ZERO_POINT.polar(self.origin.distance(geo_point), self.direction - self.origin.direction(geo_point))
