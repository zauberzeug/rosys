from rosys.driving.driver import Carrot
from rosys.geometry import Point, Pose, Spline


def make_spline() -> Spline:
    return Spline.from_points(Point(x=0, y=0), Point(x=1, y=0))


def test_carrot_advances():
    # hook behind carrot → t advances, keep driving
    carrot = Carrot(spline=make_spline())
    hook = Point(x=0, y=0)
    result = carrot.move(hook, distance=0.1)
    assert result
    assert carrot.t > 0


def test_carrot_clamps_at_end():
    # t must not overshoot 1.0 even with large carrot distance
    carrot = Carrot(spline=make_spline())
    hook = Point(x=0, y=0)
    carrot.move(hook, distance=2.0)
    assert carrot.t == 1.0


def test_stops_at_spline_end():
    # hook at spline endpoint → stop driving
    carrot = Carrot(spline=make_spline(), t=0.99)
    hook = Point(x=1, y=0)
    result = carrot.move(hook, distance=0.01)
    assert not result


def test_pose_override_keeps_driving():
    # hook at end but robot still mid-spline → keep driving (backward driving case)
    carrot = Carrot(spline=make_spline(), t=0.99)
    hook = Point(x=1, y=0)
    pose = Pose(x=0.5, y=0)
    result = carrot.move(hook, distance=0.01, pose=pose)
    assert result


def test_pose_override_stops():
    # both hook and robot at endpoint → stop driving
    carrot = Carrot(spline=make_spline(), t=0.99)
    hook = Point(x=1, y=0)
    pose = Pose(x=1, y=0)
    result = carrot.move(hook, distance=0.01, pose=pose)
    assert not result
