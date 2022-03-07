import numpy as np
from rosys.world import Calibration, Camera, Point3d
from rosys.test import approx


def demo_data() -> tuple[Camera, list[Point3d]]:
    cam = Camera.create_perfect_camera(x=0.1, y=0.2, z=3, tilt_x=np.deg2rad(10), tilt_y=np.deg2rad(20))
    world_points = [
        Point3d(x=x, y=y, z=z)
        for x in [-1.0, 0.0, 1.0]
        for y in [-1.0, 0.0, 1.0]
        for z in [-1.0, 0.0, 1.0]
    ]
    return cam, world_points


def test_calibration_from_points():
    cam, world_points = demo_data()
    image_size = cam.calibration.intrinsics.size

    image_points = [cam.calibration.project_to_image(p) for p in world_points]
    focal_length = cam.calibration.intrinsics.matrix[0][0]
    calibration = Calibration.from_points(world_points, image_points, image_size, focal_length)

    approx(calibration.intrinsics.matrix, cam.calibration.intrinsics.matrix)
    approx(calibration.intrinsics.rotation.R, cam.calibration.intrinsics.rotation.R)
    approx(calibration.extrinsics.translation, cam.calibration.extrinsics.translation)
    approx(calibration.extrinsics.yaw, cam.calibration.extrinsics.yaw)
    approx(calibration.extrinsics.tilt.R, cam.calibration.extrinsics.tilt.R)


def test_projection():
    cam, world_points = demo_data()
    for world_point in world_points:
        image_point = cam.calibration.project_to_image(world_point)
        world_point_ = cam.calibration.project_from_image(image_point, target_height=world_point.z)
        assert np.allclose(world_point.tuple, world_point_.tuple, atol=1e-6)


def test_array_projection():
    cam, world_points = demo_data()
    world_points = [p for p in world_points if p.z == 1]

    world_point_array = np.array([p.tuple for p in world_points])
    image_point_array = cam.calibration.project_array_to_image(world_point_array)
    for i, world_point in enumerate(world_points):
        image_point = cam.calibration.project_to_image(world_point)
        assert np.allclose(image_point.tuple, image_point_array[i])
    world_point_array_ = cam.calibration.project_array_from_image(image_point_array, target_height=1)
    assert np.allclose(world_point_array, world_point_array_, atol=1e-6)
