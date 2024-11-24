from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from .point import Point
from .pose import Pose

if TYPE_CHECKING:
    from .geo_point import Fixpoint, GeoPoint
    from .geo_pose import GeoPose


ZERO_POINT = Point(x=0, y=0)


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

    # TODO: keep here or move to GeoPoint?
    def point_to_geo(self, point: Point | Pose) -> GeoPoint:
        """Convert a local point to a global point (latitude, longitude; all in degrees)."""
        if not self.is_set:
            raise MissingGeoReferenceError
        assert self.origin is not None
        return self.origin.polar(ZERO_POINT.distance(point), self.direction - ZERO_POINT.direction(point))

    # TODO: keep here or move to GeoPoint?
    def point_to_local(self, geo_point: GeoPoint | GeoPose) -> Point:
        """Convert a global point to a local point."""
        if not self.is_set:
            raise MissingGeoReferenceError
        assert self.origin is not None
        return ZERO_POINT.polar(self.origin.distance(geo_point), self.direction - self.origin.direction(geo_point))

    @property
    def degree_tuple(self) -> tuple[float, float, float]:
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
        if not self.is_set:
            raise MissingGeoReferenceError
        assert self.origin is not None
        lat_deg, lon_deg, direction_deg = self.degree_tuple
        return f'GeoReference(lat={lat_deg:.6f}˚, lon={lon_deg:.6f}˚, heading={direction_deg:.1f}˚)'


# TODO: naming
current_geo_reference = GeoReference()
