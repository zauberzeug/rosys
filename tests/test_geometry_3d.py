import numpy as np
import pytest

from rosys.geometry import Point3d, Pose3d, Rotation
from rosys.testing import poses_equal


def test_rotation_as_quaternion():
    rotation = Rotation.from_euler(roll=0.1, pitch=0.2, yaw=0.3)
    quaternion = rotation.quaternion
    rotation_ = Rotation.from_quaternion(*quaternion)
    assert np.allclose(rotation.R, rotation_.R)


def test_resolve_frames():
    #                [P↓]
    #              ↗
    #         [B→] → [Q]
    #          ↑
    #          ↑
    # [O] →→→ [A↑]
    turn_left = Rotation.from_euler(0, 0, np.pi / 2)
    turn_right = Rotation.from_euler(0, 0, -np.pi / 2)
    turn_180 = Rotation.from_euler(0, 0, np.pi)
    A = Pose3d(x=3, y=0, z=0, rotation=turn_left).as_frame('A')
    B = Pose3d(x=2, y=0, z=0, rotation=turn_right).as_frame('B').in_frame(A)

    P = Pose3d(x=1, y=1, z=0, rotation=turn_right).in_frame(B)
    assert poses_equal(P.resolve(), Pose3d(x=4, y=3, z=0, rotation=turn_right))

    Q = Point3d(x=1, y=0, z=0).in_frame(B)
    assert Q.resolve() == Point3d(x=4, y=2, z=0)

    P_in_A = P.relative_to(A)
    assert P_in_A.frame_id is None
    assert poses_equal(P_in_A, Pose3d(x=3, y=-1, z=0, rotation=turn_180))


def test_check_for_frame_cycles():
    A = Pose3d().as_frame('A')
    B = Pose3d().as_frame('B').in_frame(A)
    with pytest.raises(ValueError, match='Cannot place frame "A" in frame "B" because it creates a cycle'):
        A.in_frame(B)


def test_inverse_pose():
    pose = Pose3d(x=1, y=2, z=3, rotation=Rotation.from_euler(roll=0.1, pitch=0.2, yaw=0.3))
    assert poses_equal(pose.inverse().inverse(), pose)
    assert poses_equal(pose @ pose.inverse(), Pose3d())
