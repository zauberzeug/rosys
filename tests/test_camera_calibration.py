import copy

import cv2
import numpy as np
import pytest

from rosys.geometry import Point, Point3d, Pose3d
from rosys.geometry.object3d import frame_registry
from rosys.testing import approx
from rosys.vision import CalibratableCamera, Calibration, Image, ImageSize
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
    image_points = [p for p in image_points if p is not None]
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
    image_points = [p for p in image_points if p is not None]
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
    image_points = [p for p in image_points if p is not None]
    focal_length = cam.calibration.intrinsics.matrix[0][0]
    calibration = Calibration.from_points(world_points, image_points, image_size=image_size, f0=focal_length,
                                          camera_model=CameraModel.FISHEYE)

    approx(calibration.intrinsics.matrix, cam.calibration.intrinsics.matrix)
    approx(calibration.extrinsics.translation, cam.calibration.extrinsics.translation)
    approx(calibration.extrinsics.rotation.R, cam.calibration.extrinsics.rotation.R, abs=1e-6)


def test_omnidirectional_calibration_from_points():
    cam, world_points_ = demo_omnidirectional_data()
    assert cam.calibration is not None
    image_size = cam.calibration.intrinsics.size

    def translated_calibrations(base_calibration: Calibration, n=6):
        for dz in np.linspace(0, 4, n):
            calibration = copy.deepcopy(base_calibration)
            calibration.extrinsics.z += dz
            yield calibration

    n_views = 10

    image_points = [[calib.project_to_image(p) for p in world_points_]
                    for calib in translated_calibrations(cam.calibration, n_views)]
    world_points = [world_points_ for _ in range(n_views)]
    assert not any(p is None for view in image_points for p in view)
    image_points = [[p for p in view if p is not None] for view in image_points]
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

    heights = set(p.z for p in world_points)

    for height in heights:
        world_points_at_height = [p for p in world_points if p.z == height]
        image_points: list[Point] = cam.calibration.project_to_image(world_points_at_height)
        assert not any(p is None for p in image_points)
        reprojected_world_points_: list[Point3d] = cam.calibration.project_from_image(
            image_points, target_height=height)
        assert not any(p is None for p in reprojected_world_points_)
        assert np.allclose([p.tuple for p in world_points_at_height], [p.tuple for p in reprojected_world_points_], atol=1e-6), \
            f'batch projection of world points at height {height} did not reproject back to the original points'

    for i, world_point in enumerate(world_points):
        image_point = cam.calibration.project_to_image(world_point)
        assert image_point is not None
        world_point_ = cam.calibration.project_from_image(image_point, target_height=world_point.z)
        assert world_point_ is not None
        assert np.allclose(world_point.tuple, world_point_.tuple,
                           atol=1e-6), f'world_point {i} did not reproject back to the original point'


def test_projection_of_empty_array():
    cam, _ = demo_data()
    assert cam.calibration.project_to_image([]) == []
    assert cam.calibration.project_from_image([]) == []


def test_projection_with_custom_coordinate_frame():
    cam, world_points = demo_data()
    assert cam.calibration is not None

    frame_registry.clear()
    cam_frame = Pose3d(x=0.03, y=-0.02, z=0.0).as_frame('cam')
    cam.calibration.extrinsics.in_frame(cam_frame)

    heights = set(p.z for p in world_points)

    for height in heights:
        world_points_at_height = [p for p in world_points if p.z == height]
        image_points: list[Point] = cam.calibration.project_to_image(world_points_at_height)
        assert not any(p is None for p in image_points)
        reprojected_world_points_: list[Point3d] = cam.calibration.project_from_image(
            image_points, target_height=height)
        assert not any(p is None for p in reprojected_world_points_)
        assert np.allclose([p.tuple for p in world_points_at_height], [p.tuple for p in reprojected_world_points_], atol=1e-6), \
            f'batch projection of world points at height {height} did not reproject back to the original points'

    for world_point in world_points:
        image_point = cam.calibration.project_to_image(world_point)
        assert image_point is not None
        world_point_ = cam.calibration.project_from_image(image_point, target_height=world_point.z)
        assert world_point_ is not None
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


def test_undistort_points():
    """Test """
    cam, _ = demo_data()
    assert cam.calibration is not None

    # Test with list[Point]
    points = [Point(x=120, y=90), Point(x=180, y=210), Point(x=240, y=150)]
    undistorted_points = cam.calibration.undistort_points(points)
    assert isinstance(undistorted_points, list)
    assert all(isinstance(p, Point) for p in undistorted_points)

    # single point
    undistorted_point = cam.calibration.undistort_points(points[0])
    assert isinstance(undistorted_point, Point)

    # Test with numpy array
    points_array = np.array([point.tuple for point in points], dtype=np.float32)
    undistorted_array = cam.calibration.undistort_points(points_array)
    assert isinstance(undistorted_array, np.ndarray)
    assert undistorted_array.shape == (3, 2)

    distorted_array = cam.calibration.distort_points(undistorted_array)
    assert np.allclose(points_array, distorted_array, atol=1e-6)

    # Test with crop parameter
    undistorted_points_crop = cam.calibration.undistort_points(points, crop=True)
    assert len(undistorted_points_crop) == len(points)

    # check that point, points and array have the same effect
    assert np.allclose(np.array([p.tuple for p in undistorted_points]), undistorted_array)
    assert np.allclose(undistorted_points[0].tuple, undistorted_point.tuple)


def test_undistort_image_with_crop():
    cam, _ = demo_data()
    assert cam.calibration is not None

    # Create a test image
    test_image = Image(
        camera_id='1',
        size=ImageSize(width=800, height=600),
        time=0.0,
        data=cv2.imencode('.jpg', np.zeros((600, 800, 3), dtype=np.uint8))[1].tobytes(),
        is_broken=False,
        tags=set()
    )

    # Test without crop
    undistorted = cam.calibration.undistort_image(test_image, crop=False)
    assert undistorted.size == test_image.size

    # Test with crop
    undistorted_crop = cam.calibration.undistort_image(test_image, crop=True)
    assert undistorted_crop.size.width <= test_image.size.width
    assert undistorted_crop.size.height <= test_image.size.height


@pytest.mark.parametrize('crop', [True, False])
def test_distort_points_overload(crop: bool):
    cam, _ = demo_data()
    assert cam.calibration is not None

    points = [Point(x=100.0, y=100.0), Point(x=200.0, y=200.0), Point(x=300.0, y=300.0)]

    # list of points
    distorted_points = cam.calibration.distort_points(points, crop=crop)
    assert isinstance(distorted_points, list)
    assert all(isinstance(p, Point) for p in distorted_points)

    # single point
    distorted_point = cam.calibration.distort_points(points[0], crop=crop)
    assert isinstance(distorted_point, Point)

    # numpy array
    points_array = np.array([point.tuple for point in points], dtype=np.float32).reshape((-1, 2))
    distorted_array = cam.calibration.distort_points(points_array, crop=crop)
    assert isinstance(distorted_array, np.ndarray)
    assert distorted_array.shape == (3, 2)

    assert np.allclose(np.array([p.tuple for p in distorted_points]), distorted_array, atol=1e-4)
    assert np.allclose(distorted_points[0].tuple, distorted_point.tuple, atol=1e-4)


@pytest.mark.parametrize('distortion', [
    [0.11, -0.12, 0.13, -0.14],
    [0.11, -0.12, 0.13, -0.14, 0.15],
    [20.13, 5.89, 0.0001, -0.00025, 0.0706, 21.58, 13.108, 0.8059],
    [20.13, 5.89, 0.0001, -0.00025, 0.0706, 21.58, 13.108, 0.8059, 0.0001, -0.0003, 0.0678, -0.0012],
    [20.13, 5.89, 0.0001, -0.00025, 0.0706, 21.58, 13.108, 0.8059, 0.0001, -0.0003, 0.0678, -0.0012, 0.0004, -0.0005],
])
def test_distort_points_pinhole(distortion: list[float]):
    cam = CalibratableCamera(id='1')
    cam.set_perfect_calibration(z=4)
    assert cam.calibration is not None

    cam.calibration.intrinsics.distortion = distortion
    cam.calibration.intrinsics.model = CameraModel.PINHOLE

    points = np.array([[100, 100], [200, 200], [300, 300], [400, 400]], dtype=np.float32)
    undistorted_points = cam.calibration.undistort_points(points)
    redistorted_points = cam.calibration.distort_points(undistorted_points)
    assert np.allclose(points, redistorted_points, atol=0.4)


@pytest.mark.parametrize('crop', [True, False])
def test_distort_points_fisheye(crop: bool):
    cam = CalibratableCamera(id='1')
    cam.set_perfect_calibration(z=4, roll=np.deg2rad(180 + 10))
    assert cam.calibration is not None

    cam.calibration.intrinsics.distortion = [-0.014, -0.0023, 0.001, 0.001]
    cam.calibration.intrinsics.model = CameraModel.FISHEYE

    points = np.array([[100, 100], [200, 200], [300, 300], [400, 400]], dtype=np.float32)
    undistorted_points = cam.calibration.undistort_points(points, crop=crop)
    redistorted_points = cam.calibration.distort_points(undistorted_points, crop=crop)
    assert np.allclose(points, redistorted_points, atol=1e-6)
