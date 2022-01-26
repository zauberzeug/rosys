import numpy as np
from rosys.world import Calibration, Camera, Point3d
from rosys.test.helper import approx


def test_calibration_from_points():
    cam = Camera.create_perfect_camera(x=0.1, y=0.2, z=3, tilt_x=np.deg2rad(-170), tilt_y=np.deg2rad(10))
    image_size = cam.calibration.intrinsics.size

    world_points = [
        Point3d(x=x, y=y, z=z)
        for x in [-1.0, 0.0, 1.0]
        for y in [-1.0, 0.0, 1.0]
        for z in [-1.0, 0.0, 1.0]
    ]
    image_points = [cam.calibration.project_to_image(p) for p in world_points]

    focal_length = cam.calibration.intrinsics.matrix[0][0]
    calibration = Calibration.from_points(world_points, image_points, image_size, focal_length)

    approx(calibration.intrinsics.matrix, cam.calibration.intrinsics.matrix)
    approx(calibration.intrinsics.rotation.R, cam.calibration.intrinsics.rotation.R)
    approx(calibration.extrinsics.translation, cam.calibration.extrinsics.translation)
    approx(calibration.extrinsics.yaw, cam.calibration.extrinsics.yaw)
    approx(calibration.extrinsics.tilt.R, cam.calibration.extrinsics.tilt.R)
