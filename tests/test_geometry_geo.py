import math

import pytest

from rosys.geometry import Fixpoint, GeoPoint, GeoReference, Point
from rosys.geometry.geo_point import R

equator_circumference = R * 2 * math.pi
one_degree_at_equator = equator_circumference / 360


def test_distance():
    point = GeoPoint.from_degrees(lat=0, lon=0)
    assert point.distance(point) == 0
    assert point.distance(GeoPoint.from_degrees(lat=0, lon=1)) == pytest.approx(one_degree_at_equator, abs=1e-8)


def test_direction():
    point_1 = GeoPoint.from_degrees(lat=0, lon=0)
    point_west = GeoPoint.from_degrees(lat=0, lon=-1)
    point_east = GeoPoint.from_degrees(lat=0, lon=1)
    # TODO: is the direction correct?
    assert point_1.direction(point_west) == pytest.approx(math.radians(90), abs=1e-8)
    assert point_1.direction(point_east) == pytest.approx(math.radians(-90), abs=1e-8)


def test_polar():
    point_1 = GeoPoint.from_degrees(lat=52, lon=7)
    point_2 = point_1.polar(distance=1.5, direction=math.radians(90))
    assert point_1.distance(point_2) == pytest.approx(1.5, abs=1e-8)
    assert point_1.direction(point_2) == pytest.approx(math.radians(90), abs=1e-8)


def test_shifted():
    point_1 = GeoPoint.from_degrees(lat=52, lon=7)
    for distance in (1.5, 100, 10_000_000):
        point_2 = point_1.shifted(point=Point(x=0, y=distance))
        assert point_1.distance(point_2) == pytest.approx(distance, abs=1e-8)


def test_reference_from_fixpoints():
    fixpoint_1 = Fixpoint(local_point=Point(x=0, y=0), geo_point=GeoPoint.from_degrees(lat=52, lon=7))
    fixpoint_2 = Fixpoint(Point(x=0, y=one_degree_at_equator), GeoPoint.from_degrees(lat=52, lon=6))
    reference = GeoReference.from_two_fixpoints(fixpoint_1, fixpoint_2)
    assert reference.origin.lat == pytest.approx(fixpoint_1.geo_point.lat, abs=1e-8)
    assert reference.origin.lon == pytest.approx(fixpoint_1.geo_point.lon, abs=1e-8)
    assert reference.direction == pytest.approx(0, abs=1e-2)


def test_cartesian():
    reference = GeoReference(GeoPoint.from_degrees(lat=0, lon=0), direction=0)
    point_west = GeoPoint.from_degrees(lat=0, lon=-1)
    point_east = GeoPoint.from_degrees(lat=0, lon=1)
    point_west_local = reference.point_to_local(point_west)
    point_east_local = reference.point_to_local(point_east)
    assert point_west_local.y == pytest.approx(one_degree_at_equator, abs=1e-8)
    assert point_east_local.y == pytest.approx(-one_degree_at_equator, abs=1e-8)
