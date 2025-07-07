import numpy as np
import pytest

from rosys.geometry import Point3d, Rotation
from rosys.vision import CalibratableCamera, SpatialResection


def demo_data(parallel_lines: bool = False) -> tuple[CalibratableCamera, np.ndarray, np.ndarray]:
    cam = CalibratableCamera(id='1')
    cam.set_perfect_calibration(x=0.1, y=0.2, z=3, roll=np.deg2rad(180+10), pitch=np.deg2rad(20),
                                yaw=np.deg2rad(30), distortion=[0.0, 0.0, 0.0, 0.0, 0.0])

    world_points = np.array([
        [x, y, z]
        for x in [-1.0, 0.0, 1.0]
        for y in [-1.0, 0.0, 1.0]
        for z in [-1.0, 0.0, 1.0]]
    ).astype(np.float32)

    n_points = world_points.shape[0]

    world_lines = np.zeros((n_points, 6), dtype=np.float32)
    world_lines[:, :3] = world_points.copy()

    np.random.seed(42)
    if parallel_lines:
        directions = np.array([[0.1, 0.1, 0.1]]*n_points)
        directions[::2] *= -1
    else:
        directions = np.random.randn(n_points, 3)/10
    offset_factors = np.random.randn(n_points, 1)/10

    world_lines[:, 3:] = directions
    world_lines[:, :3] += directions*offset_factors

    return cam, world_points, world_lines


@pytest.mark.parametrize('parallel_lines', (False, True))
def test_spatial_resection_with_line_points(parallel_lines: bool):
    """Test the spatial resection with line points.

    The resection is performed in two ways:
    - with non-parallel lines only.
    - with parallel lines and additional world points.
    """

    cam, world_points, world_lines = demo_data(parallel_lines=parallel_lines)
    calibration = cam.calibration
    assert calibration is not None
    intrinsics = calibration.intrinsics

    # Generate image points
    image_line_points = calibration.project_to_image(world_points)

    # Starting values for the LSA
    p0 = Point3d(x=0.0, y=0.0, z=3)
    r0 = Rotation.from_euler(np.pi, 0, 0)
    s0 = np.linspace(-1, 1, len(world_lines))

    sr = SpatialResection(intrinsics)

    if parallel_lines:
        result = sr.lsa_with_lines(
            p0=p0, r0=r0, s0=s0,
            world_lines=world_lines,
            image_line_points=image_line_points,
            world_points=world_points[3:],
            image_points=image_line_points[3:])
    else:
        result = sr.lsa_with_lines(
            p0=p0, r0=r0, s0=s0,
            world_lines=world_lines,
            image_line_points=image_line_points)

    # Check the camera pose
    camera_pose = result.camera_pose

    ground_truth_translation = calibration.extrinsics.point_3d
    ground_truth_rotation = calibration.extrinsics.rotation

    assert np.allclose(camera_pose.point_3d.array, ground_truth_translation.array, atol=0.001)
    assert np.allclose(camera_pose.rotation.quaternion, ground_truth_rotation.quaternion, atol=0.001)

    # Check the estimated object space points on lines
    for line_point, world_point in zip(result.estimated_points_on_lines or [], world_points, strict=False):
        assert np.allclose(line_point.array, world_point, atol=5)
