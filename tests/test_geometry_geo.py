import math

import pytest

from rosys.geometry import Fixpoint, GeoPoint, GeoPose, GeoReference, Point
from rosys.geometry.geo import RADIUS

CIRCUMFERENCE = RADIUS * 2 * math.pi
ONE_DEGREE_ARC_LENGTH = CIRCUMFERENCE / 360


def test_distance():
    point = GeoPoint.from_degrees(lat=0, lon=0)
    assert point.distance(point) == 0
    assert point.distance(GeoPoint.from_degrees(lat=0, lon=1)) == pytest.approx(ONE_DEGREE_ARC_LENGTH)


@pytest.mark.usefixtures('geo_reference')
def test_direction() -> None:
    point_origin = GeoPoint.from_degrees(lat=0, lon=0)
    point_east = GeoPoint.from_degrees(lat=0, lon=1)
    point_west = GeoPoint.from_degrees(lat=0, lon=-1)
    assert point_origin.direction(point_east) == pytest.approx(math.radians(90))
    assert point_origin.direction(point_west) == pytest.approx(math.radians(-90))


@pytest.mark.parametrize('angle_degrees', (0, 45, 90, 135, 180, -45, -90, -135, -180))
def test_polar(angle_degrees: float) -> None:
    point_1 = GeoPoint.from_degrees(lat=0, lon=0)
    angle = math.radians(angle_degrees)
    point_2 = point_1.polar(distance=1, direction=angle)
    assert point_1.distance(point_2) == pytest.approx(1)
    assert point_1.direction(point_2) == pytest.approx(angle)


@pytest.mark.usefixtures('geo_reference')
@pytest.mark.parametrize('dy', (1, 100, ONE_DEGREE_ARC_LENGTH, 10_000_000))
def test_shifted(dy: float) -> None:
    point_1 = GeoPoint.from_degrees(lat=0, lon=0)
    point_2 = point_1.shifted(x=2, y=dy)
    distance = math.sqrt(4 + dy**2)
    assert point_1.distance(point_2) == pytest.approx(distance)
    point_2_cartesian = point_2.cartesian()
    assert Point(x=0, y=0).distance(point_2_cartesian) == pytest.approx(distance)
    assert point_2_cartesian.x == pytest.approx(2)
    assert point_2_cartesian.y == pytest.approx(dy)


def test_reference_from_fixpoints() -> None:
    fixpoint_1 = Fixpoint(Point(x=0, y=0), GeoPoint.from_degrees(lat=0, lon=1))
    fixpoint_2 = Fixpoint(Point(x=0, y=ONE_DEGREE_ARC_LENGTH), GeoPoint.from_degrees(lat=0, lon=0))
    reference = GeoReference.from_two_fixpoints(fixpoint_1, fixpoint_2)
    assert fixpoint_1.geo_point is not None
    assert reference.origin.lat == fixpoint_1.geo_point.lat
    assert reference.origin.lon == fixpoint_1.geo_point.lon
    assert reference.direction == 0


@pytest.mark.usefixtures('geo_reference')
def test_update_reference() -> None:
    point_1 = GeoPoint.from_degrees(lat=0, lon=1)
    assert GeoReference.current is not None
    assert GeoReference.current.origin.distance(point_1) == pytest.approx(ONE_DEGREE_ARC_LENGTH)

    new_geo_reference = GeoReference(GeoPoint.from_degrees(lat=0, lon=1))
    GeoReference.update_current(new_geo_reference)
    assert GeoReference.current.origin.lat == new_geo_reference.origin.lat
    assert GeoReference.current.origin.lon == new_geo_reference.origin.lon
    assert GeoReference.current.origin.distance(point_1) == 0


@pytest.mark.usefixtures('geo_reference')
@pytest.mark.parametrize('direction', ('east', 'west'))
def test_point_cartesian(direction: str) -> None:
    lon_degrees = 0.001 if direction == 'east' else -0.001
    point = GeoPoint.from_degrees(lat=0, lon=lon_degrees)
    point_local = point.cartesian()
    assert point_local.x == pytest.approx(0)
    assert point_local.y == pytest.approx(-lon_degrees * ONE_DEGREE_ARC_LENGTH)
    assert GeoPoint.from_point(point_local).distance(point) == pytest.approx(0)


@pytest.mark.usefixtures('geo_reference')
@pytest.mark.parametrize('heading_degrees', (0, 45, 90, 179, -45, -90, -179))
def test_pose_cartesian(heading_degrees: int) -> None:
    pose = GeoPose.from_degrees(lat=0, lon=0.001, heading=heading_degrees).cartesian()
    assert pose.x == pytest.approx(0)
    assert pose.y == pytest.approx(-0.001 * ONE_DEGREE_ARC_LENGTH)
    assert pose.yaw == pytest.approx(math.radians(-heading_degrees))
