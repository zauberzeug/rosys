import pytest
from rosys.world import Pose, PoseStep


def test_pose():
    pose0 = Pose(x=1, y=2, yaw=0)
    pose1 = Pose(x=4, y=6, yaw=0)
    assert pose0.distance(pose1) == pytest.approx(5.0, 0.01)
    assert pose0.projected_distance(pose1) == pytest.approx(3.0, 0.01)


def test_adding_steps():
    pose = Pose(x=1, y=2, yaw=0, time=10.0)
    assert pose + PoseStep(linear=0.1, angular=0, time=11.0) == Pose(x=1.1, y=2.0, yaw=0.0, time=11.0)
    assert pose + PoseStep(linear=0, angular=0.2, time=12.0) == Pose(x=1.0, y=2.0, yaw=0.2, time=12.0)

    pose += PoseStep(linear=0.1, angular=0.2, time=13.0)
    assert pose == Pose(x=1.1, y=2.0, yaw=0.2, time=13.0)
