import pytest
import numpy as np
from world.robot import Pose
from world.world import World

global_world: World = None


def set_global_world(world: World):
    global global_world
    global_world = world


def assert_pose(x: float, y: float, *, yaw: int = None, linear_tolerance: int = 0.01, angular_tolerance: int = 1):
    """yaw and tolerances are in degree for better readability"""
    pose = global_world.robot.pose
    assert pose.x == pytest.approx(x, abs=linear_tolerance)
    assert pose.y == pytest.approx(y, abs=linear_tolerance)

    assert np.rad2deg(pose.yaw) == pytest.approx(yaw, abs=angular_tolerance)


def drive(m_per_s: float, *, deg_per_s: int = 0):
    global_world.robot.drive(m_per_s, rad_per_s=np.deg2rad(deg_per_s))
