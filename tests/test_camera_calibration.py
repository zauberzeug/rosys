import copy

import numpy as np

from rosys.geometry import Point3d, Pose3d
from rosys.geometry.object3d import frame_registry
from rosys.testing import approx
from rosys.vision import CalibratableCamera, Calibration
from rosys.vision.calibration import CameraModel, OmnidirParameters


def demo_data() -> tuple[CalibratableCamera, list[Point3d]]:
    cam = CalibratableCamera(id='1')
    cam.set_perfect_calibration(x=0.1, y=0.2, z=3, roll=np.deg2rad(180+10), pitch=np.deg2rad(20), yaw=np.deg2rad(30))

    world_points = [
        Point3d(x=x, y=y, z=z)
        for x in [-1.0, 0.0, 1.0]
        for y in [-1.0, 0.0, 1.0]
        for z in [-1.0, 0.0, 1.0]
    ]
    return cam, world_points


def demo_fisheye_data() -> tuple[CalibratableCamera, list[Point3d]]:
    cam = CalibratableCamera(id='1')
    cam.set_perfect_calibration(width=800, height=600, x=0.1, y=0.2, z=3,
                                roll=np.deg2rad(180+10), pitch=np.deg2rad(20), yaw=np.deg2rad(30))
    assert cam.calibration

    cam.calibration.intrinsics.distortion = [0.1, 0.4, -0.5, 0.2]
    cam.calibration.intrinsics.model = CameraModel.FISHEYE
    world_points = [
        Point3d(x=x, y=y, z=z)
        for x in [-2.0, -0.5, 0.0, 0.5, 2.0]
        for y in [-1.0, -0.5, 0.0, 0.5, 1.0]
        for z in [-1.0, 0.0, 1.0]
    ]
    return cam, world_points


def demo_omnidirectional_data() -> tuple[CalibratableCamera, list[Point3d]]:
    cam = CalibratableCamera(id='1')
    cam.set_perfect_calibration(width=800, height=600, focal_length=700,
                                x=0.1, y=0.2, z=3,
                                roll=np.deg2rad(180+10), pitch=np.deg2rad(5), yaw=np.deg2rad(30))
    assert cam.calibration

    cam.calibration.intrinsics.distortion = [-0.3, 0.06, -0.001, 0.0002]
    cam.calibration.intrinsics.omnidir_params = OmnidirParameters(xi=0.7)
    cam.calibration.intrinsics.model = CameraModel.OMNIDIRECTIONAL

    world_points = [Point3d(x=x, y=y, z=1)
                    for x in np.linspace(-3.0, 3.0, 8)
                    for y in np.linspace(-3.0, 3.0, 8)
                    ]
    return cam, world_points


def test_calibration_from_points():
    cam, world_points = demo_data()
    assert cam.calibration is not None
    image_size = cam.calibration.intrinsics.size

    image_points = [cam.calibration.project_to_image(p) for p in world_points]
    assert not any(p is None for p in image_points)
    focal_length = cam.calibration.intrinsics.matrix[0][0]

    calibration = Calibration.from_points(world_points, image_points, image_size=image_size, f0=focal_length)

    approx(calibration.intrinsics.matrix, cam.calibration.intrinsics.matrix)
    approx(calibration.extrinsics.translation, cam.calibration.extrinsics.translation)
    approx(calibration.extrinsics.rotation.R, cam.calibration.extrinsics.rotation.R, abs=1e-6)


def test_calibration_with_custom_coordinate_frame():
    cam, world_points = demo_data()
    assert cam.calibration is not None
    image_size = cam.calibration.intrinsics.size

    frame_registry.clear()
    cam_frame = Pose3d(x=0.03, y=-0.02, z=0.0).as_frame('cam')
    cam.calibration.extrinsics.in_frame(cam_frame)

    image_points = [cam.calibration.project_to_image(p) for p in world_points]
    assert not any(p is None for p in image_points)
    focal_length = cam.calibration.intrinsics.matrix[0][0]

    calibration = Calibration.from_points(world_points, image_points, image_size=image_size, f0=focal_length)

    cam_to_world_refernce = cam.calibration.extrinsics.resolve()

    approx(calibration.intrinsics.matrix, cam.calibration.intrinsics.matrix)
    approx(calibration.extrinsics.translation, cam_to_world_refernce.translation)
    approx(calibration.extrinsics.rotation.R, cam_to_world_refernce.rotation.R, abs=1e-6)


def test_fisheye_calibration_from_points():
    cam, world_points = demo_fisheye_data()
    assert cam.calibration is not None
    image_size = cam.calibration.intrinsics.size

    image_points = [cam.calibration.project_to_image(p) for p in world_points]
    assert not any(p is None for p in image_points)
    focal_length = cam.calibration.intrinsics.matrix[0][0]
    calibration = Calibration.from_points(world_points, image_points, image_size=image_size, f0=focal_length,
                                          camera_model=CameraModel.FISHEYE)

    approx(calibration.intrinsics.matrix, cam.calibration.intrinsics.matrix)
    approx(calibration.extrinsics.translation, cam.calibration.extrinsics.translation)
    approx(calibration.extrinsics.rotation.R, cam.calibration.extrinsics.rotation.R, abs=1e-6)


def test_omnidirectional_calibration_from_points():
    cam, world_points = demo_omnidirectional_data()
    assert cam.calibration is not None
    image_size = cam.calibration.intrinsics.size

    def translated_calibrations(base_calibration: Calibration, n=6):
        for dz in np.linspace(0, 4, n):
            calibration = copy.deepcopy(base_calibration)
            calibration.extrinsics.z += dz
            yield calibration

    n_views = 10

    image_points = [[calib.project_to_image(p) for p in world_points]
                    for calib in translated_calibrations(cam.calibration, n_views)]
    world_points = [world_points for _ in range(n_views)]
    assert not any(p is None for view in image_points for p in view)
    focal_length = cam.calibration.intrinsics.matrix[0][0]
    calibration = Calibration.from_points(world_points, image_points, image_size=image_size, f0=focal_length,
                                          camera_model=CameraModel.OMNIDIRECTIONAL)

    new_image_points = [calibration.project_to_image(p) for p in world_points[0]]
    rms = np.mean([np.linalg.norm((p - p_).tuple) for p, p_ in zip(image_points[0], new_image_points, strict=True)])
    approx(rms, 0, abs=1e-1)
    approx(calibration.extrinsics.rotation.R, cam.calibration.extrinsics.rotation.R, abs=1e-3)
    approx(calibration.extrinsics.translation, cam.calibration.extrinsics.translation, abs=1e-3)
    assert calibration.intrinsics.omnidir_params is not None
    approx(calibration.intrinsics.omnidir_params.rotation.R,
           cam.calibration.intrinsics.omnidir_params.rotation.R, abs=1e-3)
    approx(calibration.intrinsics.matrix, cam.calibration.intrinsics.matrix, abs=1e-1)
    approx(calibration.intrinsics.omnidir_params.xi, cam.calibration.intrinsics.omnidir_params.xi, abs=1e-1)


def test_projection():
    cam, world_points = demo_data()
    assert cam.calibration is not None

    for world_point in world_points:
        image_point = cam.calibration.project_to_image(world_point)
        assert image_point is not None
        world_point_ = cam.calibration.project_from_image(image_point, target_height=world_point.z)
        assert world_point_ is not None
        assert np.allclose(world_point.tuple, world_point_.tuple, atol=1e-6)


def test_projection_with_custom_coordinate_frame():
    cam, world_points = demo_data()
    assert cam.calibration is not None

    frame_registry.clear()
    cam_frame = Pose3d(x=0.03, y=-0.02, z=0.0).as_frame('cam')
    cam.calibration.extrinsics.in_frame(cam_frame)

    for world_point in world_points:
        image_point = cam.calibration.project_to_image(world_point)
        assert image_point is not None
        world_point_ = cam.calibration.project_from_image(image_point, target_height=world_point.z)
        assert np.allclose(world_point.tuple, world_point_.tuple, atol=1e-6)


def test_projection_from_one_frame_into_world_frame():
    cam, world_points = demo_data()
    assert cam.calibration is not None

    frame_registry.clear()
    cam_frame = Pose3d(x=1.0, y=-0.5, z=0.0).as_frame('cam')
    cam.calibration.extrinsics.in_frame(cam_frame)

    # transform world points into cam frame
    world_points_in_frame = [p.relative_to(cam_frame).in_frame(cam_frame) for p in world_points]

    for world_point, frame_point in zip(world_points, world_points_in_frame, strict=True):
        image_point_from_frame = cam.calibration.project_to_image(frame_point)
        assert image_point_from_frame is not None
        image_point_numpy_from_frame = cam.calibration.project_to_image(frame_point.array, frame=cam_frame)
        assert np.allclose(image_point_from_frame.tuple, image_point_numpy_from_frame.tolist(), atol=1e-6)
        image_point_from_world = cam.calibration.project_to_image(world_point)
        assert image_point_from_world is not None
        assert np.allclose(image_point_from_frame.tuple, image_point_from_world.tuple, atol=1e-6)

        world_point_ = cam.calibration.project_from_image(image_point_from_frame, target_height=world_point.z)
        assert world_point_ is not None
        assert np.allclose(world_point.tuple, world_point_.tuple, atol=1e-6)


def test_fisheye_projection():
    cam, world_points = demo_fisheye_data()
    assert cam.calibration is not None

    for world_point in world_points:
        image_point = cam.calibration.project_to_image(world_point)
        assert image_point is not None
        world_point_ = cam.calibration.project_from_image(image_point, target_height=world_point.z)
        assert world_point_ is not None
        assert np.allclose(world_point.tuple, world_point_.tuple, atol=1e-6)


def test_omnidirectional_projection():
    cam, world_points = demo_omnidirectional_data()
    assert cam.calibration is not None

    for world_point in world_points:
        image_point = cam.calibration.project_to_image(world_point)
        assert image_point is not None
        world_point_ = cam.calibration.project_from_image(image_point, target_height=world_point.z)
        assert world_point_ is not None
        assert np.allclose(world_point.tuple, world_point_.tuple, atol=1e-6)


def test_array_projection():
    cam, world_points = demo_data()
    assert cam.calibration is not None

    world_points = [p for p in world_points if p.z == 1]
    world_point_array = np.array([p.tuple for p in world_points])
    image_point_array = cam.calibration.project_to_image(world_point_array)

    for i, world_point in enumerate(world_points):
        image_point = cam.calibration.project_to_image(world_point)
        assert image_point is not None
        assert np.allclose(image_point.tuple, image_point_array[i])  # pylint: disable=unsubscriptable-object

    world_point_array_ = cam.calibration.project_from_image(image_point_array, target_height=1)
    assert np.allclose(world_point_array, world_point_array_, atol=1e-6)


def test_array_projection_with_custom_coordinate_frame():
    cam, world_points = demo_data()
    assert cam.calibration is not None

    frame_registry.clear()
    cam_frame = Pose3d(x=0.03, y=-0.02, z=0.0).as_frame('cam')
    cam.calibration.extrinsics.in_frame(cam_frame)

    world_points = [p for p in world_points if p.z == 1]
    world_point_array = np.array([p.tuple for p in world_points])
    image_point_array = cam.calibration.project_to_image(world_point_array)

    for i, world_point in enumerate(world_points):
        image_point = cam.calibration.project_to_image(world_point)
        assert image_point is not None
        assert np.allclose(image_point.tuple, image_point_array[i])  # pylint: disable=unsubscriptable-object

    world_point_array_ = cam.calibration.project_from_image(image_point_array, target_height=1)
    assert np.allclose(world_point_array, world_point_array_, atol=1e-6)


def test_fisheye_array_projection():
    cam, world_points = demo_fisheye_data()
    assert cam.calibration is not None

    world_points = [p for p in world_points if p.z == 1]

    world_point_array = np.array([p.tuple for p in world_points])
    image_point_array = cam.calibration.project_to_image(world_point_array)
    for i, world_point in enumerate(world_points):
        image_point = cam.calibration.project_to_image(world_point)
        assert image_point is not None
        assert np.allclose(image_point.tuple, image_point_array[i])  # pylint: disable=unsubscriptable-object
    world_point_array_ = cam.calibration.project_from_image(image_point_array, target_height=1)
    assert np.allclose(world_point_array, world_point_array_, atol=1e-6)


def test_omnidirectional_array_projection():
    cam, world_points = demo_omnidirectional_data()
    assert cam.calibration is not None

    world_points = [p for p in world_points if abs(p.z) <= 1]
    world_point_array = np.array([p.tuple for p in world_points])
    image_point_array = cam.calibration.project_to_image(world_point_array)

    for i, world_point in enumerate(world_points):
        image_point = cam.calibration.project_to_image(world_point)
        assert image_point is not None
        assert np.allclose(image_point.tuple, image_point_array[i])  # pylint: disable=unsubscriptable-object
    world_point_array_ = cam.calibration.project_from_image(image_point_array, target_height=1)
    assert np.allclose(world_point_array, world_point_array_, atol=1e-6)


def test_project_from_behind():
    cam = CalibratableCamera(id='1')
    cam.set_perfect_calibration(z=1, roll=np.deg2rad(180 + 10))
    assert cam.calibration is not None

    assert cam.calibration.project_to_image(Point3d(x=0, y=1, z=1)) is not None
    assert cam.calibration.project_to_image(Point3d(x=0, y=-1, z=1)) is None


def test_fisheye_project_from_behind():
    cam = CalibratableCamera(id='1')
    cam.set_perfect_calibration(z=1, roll=np.deg2rad(180 + 10))
    assert cam.calibration is not None

    cam.calibration.intrinsics.distortion = [0.1, 0.2, 0.3, 0.4]
    cam.calibration.intrinsics.model = CameraModel.FISHEYE
    assert cam.calibration.project_to_image(Point3d(x=0, y=1, z=1)) is not None
    assert cam.calibration.project_to_image(Point3d(x=0, y=-1, z=1)) is None


def test_omnidirectional_project_from_behind():
    cam = CalibratableCamera(id='1')
    cam.set_perfect_calibration(z=1, roll=np.deg2rad(180 + 10))
    assert cam.calibration is not None

    cam.calibration.intrinsics.distortion = [0.1, 0.2, 0.3, 0.4]
    cam.calibration.intrinsics.omnidir_params = OmnidirParameters(xi=0.8)
    cam.calibration.intrinsics.model = CameraModel.OMNIDIRECTIONAL
    assert cam.calibration.project_to_image(Point3d(x=0, y=1, z=1)) is not None
    assert cam.calibration.project_to_image(Point3d(x=0, y=-1, z=1)) is not None
