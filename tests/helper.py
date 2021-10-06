from typing import Union
import pytest
import numpy as np
from rosys.runtime import Runtime
from rosys.world.point import Point
from rosys.world.point3d import Point3d

global_runtime: Runtime = None


def set_global_runtime(runtime: Runtime):
    global global_runtime
    global_runtime = runtime


def assert_pose(
    x: float, y: float, *, deg: float = None,
    linear_tolerance: float = 0.1, deg_tolerance: float = 1.0
):
    pose = global_runtime.world.robot.prediction
    assert pose.x == pytest.approx(x, abs=linear_tolerance)
    assert pose.y == pytest.approx(y, abs=linear_tolerance)

    if deg is not None:
        assert np.rad2deg(pose.yaw) == pytest.approx(deg, abs=deg_tolerance)


def assert_point(actual: Union[Point, Point3d], expected: Union[Point, Point3d]):
    assert type(actual) == type(expected)

    assert actual.x == pytest.approx(expected.x, abs=0.1)
    assert actual.y == pytest.approx(expected.y, abs=0.1)

    if type(actual) is Point3d:
        assert actual.z == pytest.approx(expected.z, abs=0.1)
