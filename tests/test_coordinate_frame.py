import math
from copy import deepcopy

import pytest

from rosys.geometry import CoordinateFrame, Point3d, Pose3d, Rotation
from rosys.geometry.coordinate_frame_registry import coordinate_frame_registry
from rosys.testing import poses_equal


@pytest.fixture
def frame1() -> CoordinateFrame:
    return CoordinateFrame(translation=Point3d(x=1, y=2, z=3),
                           rotation=Rotation.from_euler(roll=0., pitch=0., yaw=0.))


@pytest.fixture
def frame2(frame1) -> CoordinateFrame:
    frame2 = CoordinateFrame(translation=Point3d(x=3, y=3, z=3),
                             rotation=Rotation.from_euler(roll=math.pi / 2, pitch=0., yaw=0.))
    frame2.parent_frame = frame1
    return frame2


@pytest.fixture
def pose(frame1) -> Pose3d:
    pose = Pose3d(translation=Point3d(x=1, y=1, z=1),
                  rotation=Rotation.from_euler(roll=0., pitch=0., yaw=math.pi / 2))
    pose.parent_frame = frame1
    return pose


def test_coordinate_frame_id_generation(frame1, frame2):
    assert frame1.id is not None
    assert frame2.id is not None
    assert frame1.id != frame2.id


def test_coordinate_frame_deletion(frame2):
    frame2.delete()
    assert frame2.id not in coordinate_frame_registry


def test_resolution(frame1, frame2, pose):
    assert frame1.resolve() == frame1
    assert pose.resolve(frame1) == pose

    with pytest.raises(ValueError):
        pose.resolve(frame2)

    rel_frame_2 = deepcopy(frame2)
    rel_frame_2.parent_frame = None
    rel_frame_1 = deepcopy(frame1)
    rel_frame_1.parent_frame = None

    assert frame2.resolve() == rel_frame_1 @ rel_frame_2


def test_relative_poses(frame1, frame2, pose):
    assert CoordinateFrame.common_frame(pose, frame1) == frame1
    assert CoordinateFrame.common_frame(pose, frame2) == frame1
    pose_to_frame1 = pose.relative_to(frame1)
    assert poses_equal(pose_to_frame1, pose)

    pose_to_frame2 = pose.relative_to(frame2)
    print(pose_to_frame2)
    assert poses_equal(pose_to_frame2, Pose3d(translation=Point3d(x=-2.0, y=-2.0, z=2.0),
                       rotation=Rotation.from_euler(roll=-math.pi / 4, pitch=math.pi / 2, yaw=math.pi / 4)))
    assert poses_equal(pose.relative_to(frame2, common_frame=frame1), pose_to_frame2)
    assert poses_equal(frame1 @ frame2 @ pose_to_frame2, pose.resolve())


def test_cycle_detection(frame1, frame2):
    with pytest.raises(ValueError):
        frame1.parent_frame = frame2

    assert frame1.parent_frame is None

    with pytest.raises(ValueError):
        frame1.parent_frame_id = frame2.id

    assert frame1.parent_frame is None
