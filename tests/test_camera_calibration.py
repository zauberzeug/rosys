from email.mime import image
import pytest
from rosys import world
from rosys.world import Camera, Point, Point3d
from rosys.world.calibration import Calibration


def test_calibration_from_points():
    cam = Camera.create_perfect_camera(z=2)
    stepsize = 100
    image_points = [
        Point(x=x, y=y)
        for x in range(stepsize, cam.size.width - stepsize, stepsize)
        for y in range(stepsize, cam.size.height - stepsize, stepsize)
    ]
    world_points = [cam.calibration.project_from_image(p) for p in image_points]

    calibration = Calibration.from_points(world_points, image_points, cam.size)
    assert calibration.intrinsics == cam.calibration.intrinsics
    assert calibration.extrinsics == cam.calibration.extrinsics
