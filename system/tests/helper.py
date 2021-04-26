import pytest
import numpy as np
from runtime import Runtime
import asyncio

global_runtime: Runtime = None


def set_global_runtime(runtime: Runtime):
    global global_runtime
    global_runtime = runtime


def assert_pose(
    x: float, y: float, *, deg: float = None,
    linear_tolerance: float = 0.1, deg_tolerance: float = 1.0
):
    pose = global_runtime.world.robot.pose
    assert pose.x == pytest.approx(x, abs=linear_tolerance)
    assert pose.y == pytest.approx(y, abs=linear_tolerance)

    if deg is not None:
        assert np.rad2deg(pose.yaw) == pytest.approx(deg, abs=deg_tolerance)


async def drive(linear: float, *, deg: float = 0):
    await global_runtime.esp.drive(linear, np.deg2rad(deg))


async def power(left: float, right: float):
    await global_runtime.esp.power(left, right)
