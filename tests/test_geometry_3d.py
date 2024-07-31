import numpy as np
import pytest

from rosys.geometry import Frame3d, Point3d, Pose3d, Rotation
from rosys.testing import poses_equal


def test_rotation_as_quaternion():
    rotation = Rotation.from_euler(roll=0.1, pitch=0.2, yaw=0.3)
    quaternion = rotation.quaternion
    rotation_ = Rotation.from_quaternion(*quaternion)
    assert np.allclose(rotation.R, rotation_.R)


def test_frame_id_generation():
    frame1 = Frame3d(pose=Pose3d.zero())
    frame2 = Frame3d(pose=Pose3d.zero())
    assert frame1.id is not None
    assert frame2.id is not None
    assert frame1.id != frame2.id


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
    A = Frame3d(pose=Pose3d(translation=Point3d(x=3, y=0, z=0), rotation=turn_left))
    B = Frame3d(pose=Pose3d(translation=Point3d(x=2, y=0, z=0), rotation=turn_right, frame=A))

    P = Pose3d(translation=Point3d(x=1, y=1, z=0), rotation=turn_right, frame=B)
    assert poses_equal(P.resolve(), Pose3d(translation=Point3d(x=4, y=3, z=0), rotation=turn_right))

    Q = Point3d(x=1, y=0, z=0, frame=B)
    assert Q.resolve() == Point3d(x=4, y=2, z=0)

    P_in_A = P.relative_to(A)
    assert poses_equal(P_in_A, Pose3d(translation=Point3d(x=3, y=-1, z=0), rotation=turn_180))


def test_inverse_pose():
    pose = Pose3d(translation=Point3d(x=1, y=2, z=3), rotation=Rotation.from_euler(roll=0.1, pitch=0.2, yaw=0.3))
    assert poses_equal(pose.inverse().inverse(), pose)
    assert poses_equal(pose @ pose.inverse(), Pose3d.zero())


# def test_cycle_detection():
#     frame1 = Frame3d(pose=Pose3d.zero())
#     frame2 = Frame3d(pose=Pose3d.zero(frame=frame1))
#     with pytest.raises(ValueError):
#         frame1.parent = frame2
