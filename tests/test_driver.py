import math

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
    result = carrot.move(hook, distance=2.0)
    assert carrot.t == 1.0
    assert result  # hook is still at origin → robot hasn't reached endpoint → keep driving


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


def test_carrot_clamped_pose_override_keeps_driving():
    # carrot clamped to 1.0 but robot is still mid-spline → keep driving
    carrot = Carrot(spline=make_spline())
    hook = Point(x=0, y=0)
    pose = Pose(x=0.5, y=0)
    result = carrot.move(hook, distance=2.0, pose=pose)
    assert carrot.t == 1.0
    assert result  # robot at x=0.5, not at endpoint yet → keep driving


def test_pose_at_origin():
    # Pose(x=0, y=0) is falsy in boolean context but should still be used as valid override
    carrot = Carrot(spline=make_spline(), t=0.99)
    hook = Point(x=1, y=0)
    pose = Pose(x=0, y=0)  # robot at spline start — falsy but valid
    result = carrot.move(hook, distance=0.01, pose=pose)
    assert result


def test_successive_moves():
    # simulate a drive loop: hook gradually advances, t monotonically increases
    carrot = Carrot(spline=make_spline())
    prev_t = 0.0
    steps = 0
    for i in range(20):
        hook = Point(x=i * 0.1, y=0)
        result = carrot.move(hook, distance=0.1)
        assert carrot.t >= prev_t
        prev_t = carrot.t
        steps += 1
        if not result:
            break
    assert not result
    assert steps > 1


def test_successive_moves_with_pose_override():
    # backward driving simulation: pose lags behind hook
    carrot = Carrot(spline=make_spline())
    prev_t = 0.0
    steps = 0
    for i in range(20):
        hook_x = i * 0.1
        pose_x = max(0, hook_x - 0.2)
        hook = Point(x=hook_x, y=0)
        pose = Pose(x=pose_x, y=0)
        result = carrot.move(hook, distance=0.1, pose=pose)
        assert carrot.t >= prev_t
        prev_t = carrot.t
        steps += 1
        if not result:
            break
    assert not result
    assert steps > 1


def test_curved_spline():
    # non-linear geometry: basic advance/stop behavior should be the same
    start = Pose(x=0, y=0, yaw=0)
    end = Pose(x=1, y=1, yaw=math.pi / 2)
    spline = Spline.from_poses(start, end)

    carrot = Carrot(spline=spline)
    hook = Point(x=0, y=0)
    result = carrot.move(hook, distance=0.1)
    assert result
    assert carrot.t > 0

    # at endpoint → stop
    carrot2 = Carrot(spline=spline, t=0.99)
    hook2 = Point(x=1, y=1)
    result2 = carrot2.move(hook2, distance=0.01)
    assert not result2


def test_move_by_foot_advances():
    # pose before endpoint → t advances, keep driving
    carrot = Carrot(spline=make_spline())
    pose = Pose(x=0.5, y=0)
    result = carrot.move_by_foot(pose)
    assert result
    assert carrot.t > 0
    assert carrot.t < 1.0


def test_move_by_foot_stops_at_end():
    # pose at spline endpoint → stop
    carrot = Carrot(spline=make_spline())
    pose = Pose(x=1, y=0)
    result = carrot.move_by_foot(pose)
    assert not result
    assert carrot.t == 1.0


def test_move_by_foot_never_goes_backward():
    # pose behind current t → t must not decrease (t_min constraint)
    carrot = Carrot(spline=make_spline(), t=0.5)
    pose = Pose(x=0.1, y=0)
    result = carrot.move_by_foot(pose)
    assert result
    assert carrot.t >= 0.5
