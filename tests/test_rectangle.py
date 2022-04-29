import pytest
from rosys.world import Rectangle, Point


def test_rectangle_contains_point():
    rectangle = Rectangle(x=2, y=1, width=2, height=3)
    out1 = Point(x=1, y=1)
    out2 = Point(x=5, y=3)
    in1 = Point(x=3, y=2)
    in2 = Point(x=4, y=4)

    assert not rectangle.contains(out1)
    assert not rectangle.contains(out2)
    assert rectangle.contains(in1)
    assert rectangle.contains(in2)
