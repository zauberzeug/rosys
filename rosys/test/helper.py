from typing import Union
import pytest
import numpy as np
from rosys.automations.drive_path import drive_to
from rosys.test.runtime import TestRuntime
from rosys.world.point import Point
from rosys.world.point3d import Point3d

global_runtime: TestRuntime = None


def set_global_runtime(runtime: TestRuntime):
    global global_runtime
    global camera_count
    global_runtime = runtime
    camera_count = 0


def assert_pose(x: float, y: float, *, deg: float = None, linear_tolerance: float = 0.1, deg_tolerance: float = 1.0):
    pose = global_runtime.world.robot.prediction
    assert pose.x == pytest.approx(x, abs=linear_tolerance)
    assert pose.y == pytest.approx(y, abs=linear_tolerance)

    if deg is not None:
        assert np.rad2deg(pose.yaw) == pytest.approx(deg, abs=deg_tolerance)


def assert_point(actual: Union[Point, Point3d], expected: Union[Point, Point3d], tolerance=0.1):
    assert type(actual) == type(expected)

    assert actual.x == pytest.approx(expected.x, abs=tolerance)
    assert actual.y == pytest.approx(expected.y, abs=tolerance)

    if type(actual) is Point3d:
        assert actual.z == pytest.approx(expected.z, abs=tolerance)


async def automate_drive_to(x: float, y: float):
    global_runtime.automator.replace(drive_to(global_runtime.world, global_runtime.hardware, Point(x=x, y=y)))
    await global_runtime.automator.step()  # NOTE the step will update the automation_state so we can "resume"
    await global_runtime.resume()
