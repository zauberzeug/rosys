import math

import pytest

from rosys.geometry import GeoPoint, GeoReference, Point


def test_distance_calculation():
    point = GeoPoint.from_degrees(lat=51.983159, lon=7.434212)
    assert point.distance(GeoPoint.from_degrees(lat=51.983159, lon=7.434212)) == 0
    assert 6.0 < point.distance(GeoPoint.from_degrees(lat=51.983159, lon=7.4343)) < 6.1


def test_shifted_calculation():
    geo_point = GeoPoint.from_degrees(lat=51.983159, lon=7.434212)
    geo_reference = GeoReference(origin=geo_point, direction=0)
    shifted = geo_point.shifted(Point(x=6, y=6), reference=geo_reference)
    assert shifted.lat == pytest.approx(math.radians(51.983212924295))
    assert shifted.lon == pytest.approx(math.radians(7.434124668461))


def test_feldfreund_shifted():
    geo_reference = GeoReference(GeoPoint.from_degrees(lat=51.98333489813455, lon=7.434242465994318))
    point_1 = GeoPoint.from_degrees(lat=51.98333789813455, lon=7.434242765994318)
    point_2 = point_1.shifted(point=Point(x=0, y=-1.5), reference=geo_reference)

    point_1_local = geo_reference.point_to_local(point_1)
    point_2_local = geo_reference.point_to_local(point_2)
    assert (point_1_local.x - point_2_local.x) == pytest.approx(0, abs=1e-8)
    assert (point_1_local.y - point_2_local.y) == pytest.approx(-1.5, abs=1e-8)


def test_feldfreund_distance():
    point_1 = GeoPoint(lat=0.9072804024991966, lon=0.1297520136591712)
    point_2 = GeoPoint(lat=0.9072804024991966, lon=0.1297522430263759)
    assert point_1.distance(point_2) == pytest.approx(0.9, abs=1e-8)

    point_1 = GeoPoint(lat=0.9072804057889134, lon=0.1297520139881425)
    point_2 = GeoPoint(lat=0.9072804057951533, lon=0.12975209230427967)
    assert point_1.distance(point_2) == pytest.approx(0.3, abs=1e-8)

    point_1 = GeoPoint(lat=0.9072804058295301, lon=0.1297525237621342)
    point_2 = GeoPoint(lat=0.9072804057889134, lon=0.1297520139881425)
    assert point_1.distance(point_2) == pytest.approx(2.0, abs=1e-8)
