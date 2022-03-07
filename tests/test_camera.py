import numpy as np
from rosys.actors import CameraProjector
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


def test_projector():
    cam, _ = demo_data()
    CameraProjector.update_projection(cam, rows=3, columns=4)
    assert np.allclose(cam.projection, [
        [(2.6401056200399315, -2.7431146775069135), (0.6428110258043753, -3.7739688197606194),
         (-3.507871324195061, -5.91624072397809), (-17.36746838820117, -13.069528518694367)],
        [(2.196224739529436, -0.1706616675668492), (0.38412315503644456, -0.2799447649973243),
         (-2.9432675693971544, -0.4806109792091923), (-11.05223001066152, -0.9696412521873483)],
        [(1.8490245289440204, 1.8414912905535379), (0.1961219592538731, 2.259334322339),
         (-2.5856242030237238, 2.962541680968452), (-8.25227030862712, 4.395033003953365)],
    ])
