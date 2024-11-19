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
