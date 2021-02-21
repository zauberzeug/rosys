from utilities.angle import Angle, deg
import pytest
from world.world import World

global_world: World = None


def set_global_world(world: World):
    global global_world
    global_world = world


def assert_pose(x: float, y: float, yaw: Angle = None,
                linear_tolerance: float = 0.01, angular_tolerance: Angle = deg(1)):
    """yaw and tolerances are in degree for better readability"""
    pose = global_world.robot.pose
    assert pose.x == pytest.approx(x, abs=linear_tolerance)
    assert pose.y == pytest.approx(y, abs=linear_tolerance)

    if yaw is not None:
        assert pose.yaw == pytest.approx(yaw, abs=angular_tolerance)


def drive(linear: float, angular: Angle = 0):
    global_world.robot.drive(linear, angular)
