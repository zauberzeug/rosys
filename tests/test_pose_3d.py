import copy
import math

import pytest

from rosys.geometry import Point3d, Pose3d, Rotation
from rosys.testing import approx, poses_equal


@pytest.fixture
def pose() -> Pose3d:
    return Pose3d(translation=Point3d(x=1, y=2, z=3), rotation=Rotation.from_euler(roll=0.1, pitch=0.2, yaw=0.3))


def test_inverse(pose):
    assert poses_equal(pose.inverse().inverse(), pose)
    assert poses_equal(pose @ pose.inverse(), Pose3d.zero())


def test_rotate(pose):
    pose2 = copy.deepcopy(pose)
    pose2.rotate(Rotation.from_euler(roll=0, pitch=0, yaw=math.pi / 2))
    approx(pose.translation, pose2.translation)
    reference_point = Point3d(x=0, y=1, z=0)

    approx(pose2.transform_point_to(reference_point, target_frame=pose).x,
           -1 * pose.transform_point_to(reference_point, target_frame=pose).y)
