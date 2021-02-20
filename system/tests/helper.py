import pytest
import numpy as np
from world.robot import Pose


def assert_pose(pose: Pose, x: float, y: float, *, yaw: int = None, linear_tolerance: int = 0.01, angular_tolerance: int = 1):
    """yaw and tolerances are in degree for better readability"""

    assert pose.x == pytest.approx(x, abs=linear_tolerance)
    assert pose.y == pytest.approx(y, abs=linear_tolerance)

    assert np.rad2deg(pose.yaw) == pytest.approx(yaw, abs=angular_tolerance)
