import pytest
from ..world.pose import Pose


def test_pose():

    point0 = Pose(x=1, y=2, yaw=0)
    point1 = Pose(x=4, y=6, yaw=0)
    assert point0.distance(point1) == pytest.approx(5.0, 0.01)
    assert point0.projected_distance(point1) == pytest.approx(3.0, 0.01)
