import pytest
from rosys.geometry import Line, LineSegment, Point, Pose, PoseStep, Rectangle
from rosys.test import approx


def test_line_segment_distance():
    segment = LineSegment(point1=Point(x=1, y=1), point2=Point(x=3, y=1))
    assert segment.distance(Point(x=0, y=0)) == pytest.approx(1.41, 0.01)
    assert segment.distance(Point(x=2, y=0)) == pytest.approx(1.00, 0.01)
    assert segment.distance(Point(x=2, y=1)) == pytest.approx(0.00, 0.01)
    assert segment.distance(Point(x=4, y=0)) == pytest.approx(1.41, 0.01)


def test_line_transformation():
    point1 = Point(x=1, y=2)
    point2 = Point(x=4, y=3)
    line = Line.from_points(point1, point2)

    pose = Pose(x=0.5, y=3.1, yaw=0.5)
    point1_ = pose.transform(point1)
    point2_ = pose.transform(point2)
    line_ = pose.transform_line(line)

    approx(line_, Line.from_points(point1_, point2_))


def test_pose():
    pose0 = Pose(x=1, y=2, yaw=0)
    pose1 = Pose(x=4, y=6, yaw=0)
    assert pose0.distance(pose1) == pytest.approx(5.0, 0.01)
    assert pose0.projected_distance(pose1) == pytest.approx(3.0, 0.01)


def test_pose_steps():
    pose = Pose(x=1, y=2, yaw=0, time=10.0)
    assert pose + PoseStep(linear=0.1, angular=0, time=11.0) == Pose(x=1.1, y=2.0, yaw=0.0, time=11.0)
    assert pose + PoseStep(linear=0, angular=0.2, time=12.0) == Pose(x=1.0, y=2.0, yaw=0.2, time=12.0)

    pose += PoseStep(linear=0.1, angular=0.2, time=13.0)
    assert pose == Pose(x=1.1, y=2.0, yaw=0.2, time=13.0)


def test_pose_interpolation():
    p1 = Pose(x=1.1, y=1.2, yaw=1.3)
    p2 = Pose(x=2.1, y=2.2, yaw=2.3)
    p3 = Pose(x=1.3, y=1.4, yaw=1.5)
    approx(p1.interpolate(p2, 0.2), p3)


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
