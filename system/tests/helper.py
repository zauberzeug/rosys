import pytest
import numpy as np
from world.world import World

global_world: World = None


def set_global_world(world: World):
    global global_world
    global_world = world


def assert_pose(x: float, y: float, *, yaw_deg: float = None,
                linear_tolerance: float = 0.01, angular_tolerance_deg: float = 1):
    """yaw and tolerances are in degree for better readability"""
    pose = global_world.robot.pose
    assert pose.x == pytest.approx(x, abs=linear_tolerance)
    assert pose.y == pytest.approx(y, abs=linear_tolerance)

    if yaw_deg is not None:
        assert np.rad2deg(pose.yaw) == pytest.approx(yaw_deg, abs=angular_tolerance_deg)


def drive(m_per_s: float, *, deg_per_s: float = 0):
    global_world.robot.drive(m_per_s, np.deg2rad(deg_per_s))
