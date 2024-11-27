import math

import pytest

from rosys.geometry import Fixpoint, GeoPoint, GeoPose, GeoReference, Point, current_geo_reference
from rosys.geometry.geo import R

circumference = R * 2 * math.pi
one_degree_arc_length = circumference / 360


def test_distance():
    point = GeoPoint.from_degrees(lat=0, lon=0)
    assert point.distance(point) == 0
    assert point.distance(GeoPoint.from_degrees(lat=0, lon=1)) == pytest.approx(one_degree_arc_length, abs=1e-8)


def test_direction(geo_reference: GeoReference):
    assert geo_reference.is_set
    point_1 = GeoPoint.from_degrees(lat=0, lon=0)
    point_east = GeoPoint.from_degrees(lat=0, lon=1)
    assert point_1.direction(point_east) == pytest.approx(math.radians(90), abs=1e-8)
    point_west = GeoPoint.from_degrees(lat=0, lon=-1)
    assert point_1.direction(point_west) == pytest.approx(math.radians(-90), abs=1e-8)


@pytest.mark.parametrize('angle_degrees', (0, 45, 90, 135, 180, -45, -90, -135, -180))
def test_polar(angle_degrees: float):
    point_1 = GeoPoint.from_degrees(lat=0, lon=0)
    angle = math.radians(angle_degrees)
    point_2 = point_1.polar(distance=1, direction=angle)
    assert point_1.distance(point_2) == pytest.approx(1, abs=1e-8)
    assert point_1.direction(point_2) == pytest.approx(angle, abs=1e-8)


@pytest.mark.parametrize('distance', (1, 100, one_degree_arc_length, 10_000_000))
def test_shifted(geo_reference: GeoReference, distance: float):
    assert geo_reference.is_set
    point_1 = GeoPoint.from_degrees(lat=0, lon=0)
    diagonal_distance = math.sqrt(4 + distance**2)
    point_2 = point_1.shifted(x=2, y=distance)
    assert point_1.distance(point_2) == pytest.approx(diagonal_distance, abs=1e-8)
    point_2_cartesian = point_2.cartesian()
    assert Point(x=0, y=0).distance(point_2_cartesian) == pytest.approx(diagonal_distance, abs=1e-8)
    assert point_2_cartesian.y == pytest.approx(distance, abs=1e-8)
    assert point_2_cartesian.x == pytest.approx(2, abs=1e-8)


def test_reference_from_fixpoints():
    fixpoint_1 = Fixpoint(local_point=Point(x=0, y=0), geo_point=GeoPoint.from_degrees(lat=0, lon=1))
    fixpoint_2 = Fixpoint(local_point=Point(x=0, y=one_degree_arc_length),
                          geo_point=GeoPoint.from_degrees(lat=0, lon=0))
    reference = GeoReference.from_two_fixpoints(fixpoint_1, fixpoint_2)
    assert reference.origin is not None
    assert fixpoint_1.geo_point is not None
    assert reference.origin.lat == pytest.approx(fixpoint_1.geo_point.lat, abs=1e-8)
    assert reference.origin.lon == pytest.approx(fixpoint_1.geo_point.lon, abs=1e-8)
    assert reference.direction == pytest.approx(0, abs=1e-2)


def test_update_reference(geo_reference: GeoReference):
    assert geo_reference.origin is not None
    point_1 = GeoPoint.from_degrees(lat=0, lon=1)
    assert geo_reference.origin.distance(point_1) == pytest.approx(one_degree_arc_length, abs=1e-8)
    new_geo_reference = GeoReference(GeoPoint.from_degrees(lat=0, lon=1))
    current_geo_reference.update(new_geo_reference)
    assert current_geo_reference.origin is not None
    assert new_geo_reference.origin is not None
    assert current_geo_reference.origin.lat == pytest.approx(new_geo_reference.origin.lat, abs=1e-8)
    assert current_geo_reference.origin.lon == pytest.approx(new_geo_reference.origin.lon, abs=1e-8)
    assert current_geo_reference.origin.distance(point_1) == pytest.approx(0, abs=1e-8)


@pytest.mark.parametrize('orientation', ('east', 'west'))
def test_point_cartesian(geo_reference: GeoReference, orientation: str):
    assert geo_reference.origin is not None
    orientation_sign = 1 if orientation == 'east' else -1
    point = GeoPoint.from_degrees(lat=0, lon=orientation_sign)
    point_local = point.cartesian()
    assert point_local.x == pytest.approx(0, abs=1e-8)
    assert point_local.y == pytest.approx(-orientation_sign * one_degree_arc_length, abs=1e-8)
    assert GeoPoint.from_point(point_local).distance(point) == pytest.approx(0, abs=1e-8)


# TODO: why do 45, -45 and 179, -179 fail?
@pytest.mark.parametrize('heading_degrees', (0, 45, 90, 179, -45, -90, -179))
def test_pose_cartesian(geo_reference: GeoReference, heading_degrees: int):
    assert geo_reference.origin is not None
    pose = GeoPose.from_degrees(lat=0, lon=1, heading=heading_degrees).cartesian()
    assert pose.x == pytest.approx(0, abs=1e-8)
    assert pose.y == pytest.approx(-one_degree_arc_length, abs=1e-8)
    assert pose.yaw == pytest.approx(math.radians(-heading_degrees), abs=1e-8)
