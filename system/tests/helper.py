import pytest


def assert_pose(pose, x, y, yaw, linear_tolerance=0.01, angular_tolerance=0.01):

    assert pose.x == pytest.approx(x, abs=linear_tolerance)
    assert pose.y == pytest.approx(y, abs=linear_tolerance)
    assert pose.yaw == pytest.approx(yaw, abs=angular_tolerance)
