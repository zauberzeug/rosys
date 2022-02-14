import pytest
from rosys.world import LineSegment, Point


def test_distance_to_line_segment():
    segment = LineSegment(point1=Point(x=1, y=1), point2=Point(x=3, y=1))
    assert segment.distance(Point(x=0, y=0)) == pytest.approx(1.41, 0.01)
    assert segment.distance(Point(x=2, y=0)) == pytest.approx(1.00, 0.01)
    assert segment.distance(Point(x=2, y=1)) == pytest.approx(0.00, 0.01)
    assert segment.distance(Point(x=4, y=0)) == pytest.approx(1.41, 0.01)
