import pytest
import numpy as np
from world.world import World

global_world: World = None


def set_global_world(world: World):
    global global_world
    global_world = world


def assert_pose(
    x: float, y: float, *, deg: float = None,
    linear_tolerance: float = 0.1, deg_tolerance: float = 1.0
):
    pose = global_world.robot.pose
    assert pose.x == pytest.approx(x, abs=linear_tolerance)
    assert pose.y == pytest.approx(y, abs=linear_tolerance)

    if deg is not None:
        assert np.rad2deg(pose.yaw) == pytest.approx(deg, abs=deg_tolerance)


async def drive(linear: float, *, deg: float = 0):
    await global_world.robot.drive(linear, np.deg2rad(deg))


async def power(left: float, right: float):
    await global_world.robot.power(left, right)
