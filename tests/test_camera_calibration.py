import numpy as np
from rosys.world import Calibration, Camera, Point3d


def test_calibration_from_points():
    cam = Camera.create_perfect_camera(z=-2)
    image_size = cam.calibration.intrinsics.size

    world_points = [
        Point3d(x=x, y=y, z=z)
        for x in [-1, 0, 1]
        for y in [-1, 0, 1]
        for z in [-1, 0, 1]
    ]
    image_points = [cam.calibration.project_to_image(p) for p in world_points]

    focal_length = cam.calibration.intrinsics.matrix[0][0]
    calibration = Calibration.from_points(world_points, image_points, image_size, focal_length)

    assert np.allclose(calibration.intrinsics.matrix, cam.calibration.intrinsics.matrix)
    assert np.allclose(calibration.intrinsics.rotation.R, cam.calibration.intrinsics.rotation.R)

    assert np.allclose(calibration.extrinsics.translation, cam.calibration.extrinsics.translation)
    assert np.allclose(calibration.extrinsics.yaw, cam.calibration.extrinsics.yaw)
    assert np.allclose(calibration.extrinsics.tilt.R, cam.calibration.extrinsics.tilt.R)
