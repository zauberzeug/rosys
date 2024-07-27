import copy

from rosys.geometry import Point3d, Pose3d, Rotation
from rosys.testing import approx, poses_equal

pose = Pose3d(translation=Point3d(x=1, y=2, z=3), rotation=Rotation.from_euler(roll=0.1, pitch=0.2, yaw=0.3))


def test_inverse():
    assert poses_equal(pose.inverse().inverse(), pose)
    assert poses_equal(pose @ pose.inverse(), Pose3d.zero())


def test_rotate():
    rotation = Rotation.from_euler(roll=0.4, pitch=0.5, yaw=0.6)
    pose2 = copy.deepcopy(pose)
    pose2.rotate(rotation)
    approx(pose.translation, pose2.translation)

    reference_point = Point3d(x=0, y=1, z=0)

    rotated_point = Point3d.from_tuple(rotation.matrix @ reference_point.array)
    approx(pose2.transform_point_to(reference_point, target_frame=pose), rotated_point)
