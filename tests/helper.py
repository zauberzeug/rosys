import logging
from typing import Union
import pytest
import numpy as np
from rosys.actors.camera_simulator import CameraSimulator
from rosys.actors.detector_simulator import DetectorSimulator
from rosys.runtime import Runtime
from rosys.automations.drive_path import drive_to
from rosys.world.camera import Camera
from rosys.world.point import Point
from rosys.world.point3d import Point3d

global_runtime: Runtime = None
camera_count = 0


def set_global_runtime(runtime: Runtime):
    global global_runtime
    global camera_count
    global_runtime = runtime
    camera_count = 0


def enable_tracking():
    global_runtime.world.tracking = True
    DetectorSimulator.noisy_image_points = False
    return add_camera()


def add_camera(x: float = 0, y: float = 0, z: float = 2):
    global camera_count
    camera = CameraSimulator.create_perfect_camera(x=x, y=y, z=z, mac=f'00:0{camera_count}')
    camera_count += 1
    global_runtime.world.cameras[camera.mac] = camera
    return camera


def block_sight(camera: Camera) -> None:
    logging.info(f'blocking sight to {camera}')
    detector = global_runtime.get_actor(DetectorSimulator)
    detector.simulate_sight(camera.mac, blocked=True)


def unblock_sight(camera: Camera) -> None:
    logging.info(f'unblocking sight to {camera}')
    detector = global_runtime.get_actor(DetectorSimulator)
    detector.simulate_sight(camera.mac, blocked=False)


def assert_pose(
    x: float, y: float, *, deg: float = None,
    linear_tolerance: float = 0.1, deg_tolerance: float = 1.0
):
    pose = global_runtime.world.robot.prediction
    assert pose.x == pytest.approx(x, abs=linear_tolerance)
    assert pose.y == pytest.approx(y, abs=linear_tolerance)

    if deg is not None:
        assert np.rad2deg(pose.yaw) == pytest.approx(deg, abs=deg_tolerance)


def assert_point(actual: Union[Point, Point3d], expected: Union[Point, Point3d], tolerance=0.1):
    assert type(actual) == type(expected)

    assert actual.x == pytest.approx(expected.x, abs=tolerance)
    assert actual.y == pytest.approx(expected.y, abs=tolerance)

    if type(actual) is Point3d:
        assert actual.z == pytest.approx(expected.z, abs=tolerance)


def automate_drive_to(x: float, y: float):
    global_runtime.automator.replace(drive_to(global_runtime.world, global_runtime.esp, Point(x=x, y=y)))
