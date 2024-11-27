import math

import pytest

from rosys.geometry import Fixpoint, GeoPoint, GeoReference, Point, current_geo_reference
from rosys.geometry.geo import R

equator_circumference = R * 2 * math.pi
one_degree_at_equator = equator_circumference / 360


def test_distance():
    point = GeoPoint.from_degrees(lat=0, lon=0)
    assert point.distance(point) == 0
    assert point.distance(GeoPoint.from_degrees(lat=0, lon=1)) == pytest.approx(one_degree_at_equator, abs=1e-8)


def test_direction():
    geo_reference = GeoReference(GeoPoint.from_degrees(lat=0, lon=0))
    current_geo_reference.update(geo_reference)
    point_1 = GeoPoint.from_degrees(lat=0, lon=0)
    point_east = GeoPoint.from_degrees(lat=0, lon=1)
    assert point_1.direction(point_east) == pytest.approx(math.radians(90), abs=1e-8)
    point_west = GeoPoint.from_degrees(lat=0, lon=-1)
    assert point_1.direction(point_west) == pytest.approx(math.radians(-90), abs=1e-8)


def test_polar():
    point_1 = GeoPoint.from_degrees(lat=0, lon=0)
    for angle_degrees in (0, 45, 90, 135, 180, -45, -90, -135, -180):
        angle = math.radians(angle_degrees)
        point_2 = point_1.polar(distance=1, direction=angle)
        assert point_1.distance(point_2) == pytest.approx(1, abs=1e-8)
        assert point_1.direction(point_2) == pytest.approx(angle, abs=1e-8)


def test_shifted():
    point_1 = GeoPoint.from_degrees(lat=0, lon=0)
    for distance in (1, 100, 10_000_000):
        point_2 = point_1.shifted(point=Point(x=0, y=distance))
        assert point_1.distance(point_2) == pytest.approx(distance, abs=1e-8)
        point_2_cartesian = point_2.cartesian()
        assert Point(x=0, y=0).distance(point_2_cartesian) == pytest.approx(distance, abs=1e-8)
        assert point_2_cartesian.y == pytest.approx(-distance, abs=1e-8)
        assert point_2_cartesian.x == pytest.approx(0, abs=1e-8)


def test_reference_from_fixpoints():
    fixpoint_1 = Fixpoint(local_point=Point(x=0, y=0), geo_point=GeoPoint.from_degrees(lat=0, lon=1))
    fixpoint_2 = Fixpoint(Point(x=0, y=one_degree_at_equator), GeoPoint.from_degrees(lat=0, lon=0))
    reference = GeoReference.from_two_fixpoints(fixpoint_1, fixpoint_2)
    assert reference.origin.lat == pytest.approx(fixpoint_1.geo_point.lat, abs=1e-8)
    assert reference.origin.lon == pytest.approx(fixpoint_1.geo_point.lon, abs=1e-8)
    assert reference.direction == pytest.approx(0, abs=1e-2)


def test_update_reference():
    geo_reference = GeoReference(GeoPoint.from_degrees(lat=0, lon=0))
    current_geo_reference.update(geo_reference)
    point_1 = GeoPoint.from_degrees(lat=0, lon=1)
    assert current_geo_reference.origin.distance(point_1) == pytest.approx(one_degree_at_equator, abs=1e-8)
    current_geo_reference.update(GeoReference(point_1))
    assert current_geo_reference.origin.distance(point_1) == pytest.approx(0, abs=1e-8)


def test_point_cartesian():
    reference = GeoReference(GeoPoint.from_degrees(lat=0, lon=0), direction=0)
    current_geo_reference.update(reference)
    point_east = GeoPoint.from_degrees(lat=0, lon=1)
    point_east_local = point_east.cartesian()
    assert point_east_local.x == pytest.approx(0, abs=1e-8)
    assert point_east_local.y == pytest.approx(-one_degree_at_equator, abs=1e-8)
    assert GeoPoint.from_point(point_east_local).distance(point_east) == pytest.approx(0, abs=1e-8)
    point_west = GeoPoint.from_degrees(lat=0, lon=-1)
    point_west_local = point_west.cartesian()
    assert point_west_local.x == pytest.approx(0, abs=1e-8)
    assert point_west_local.y == pytest.approx(one_degree_at_equator, abs=1e-8)
    assert GeoPoint.from_point(point_west_local).distance(point_west) == pytest.approx(0, abs=1e-8)


def test_pose_cartesian():
    # TODO
    pass
