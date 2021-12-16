import pytest
from rosys.world import Pose


def test_pose():
    pose0 = Pose(x=1, y=2, yaw=0)
    pose1 = Pose(x=4, y=6, yaw=0)
    assert pose0.distance(pose1) == pytest.approx(5.0, 0.01)
    assert pose0.projected_distance(pose1) == pytest.approx(3.0, 0.01)
