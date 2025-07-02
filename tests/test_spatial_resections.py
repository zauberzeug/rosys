import numpy as np
import pytest

from rosys.geometry import Line3d, Point, Point3d, Rotation
from rosys.vision import ImageSize, Intrinsics, SpatialResection


@pytest.mark.parametrize('num_world_points', (0, 3, 5, 9))
@pytest.mark.parametrize('use_numpy', (True, False))
def test_spatial_resection_with_line_points(num_world_points: int, use_numpy: bool):
    """
    Test the spatial resection with line points.
    The test is based on points, projected using a real camera calibration.
    The resection is performed with and without additional world points.
    Also, the API is tested using rosys data structures and numpy arrays.
    """

    world_points = np.array([
        [0.5, 0.5, 0], [-0.5, 0.5, 0],
        [-0.5, -0.5, 0], [0.5, -0.5, 0],
        [0, 0, 0.5], [1, 1, 0],
        [-1, 1, 0], [-1, -1, 0],
        [1, -1, 0], [0, 0, 1],
    ])*1000

    world_points_rosys = [
        Point3d(x=world_points[x, 0], y=world_points[x, 1], z=world_points[x, 2]) for x in range(world_points.shape[0])
    ]

    world_lines = np.array([
        [0.5, 0.5, 0, 0, 0, 1], [-0.5, 0.5, 0, 0, 1, 0],
        [-0.5, -0.5, 0, 1, 0, 0], [0, 0, 0, 0.5, -0.5, 0],
        [0, 0, 0, 0, 0, 0.5], [0, 0, 0, 1, 1, 0],
        [0, 0, 0, 1, -1, 0], [0, 0, 0, 1, 1, 0],
        [0, 0, 0, -1, 1, 0], [-1, -1, 0, 1, 1, 1],
    ])*1000

    world_lines_rosys = [
        Line3d.from_array(world_lines[x]) for x in range(world_lines.shape[0])
    ]

    image_points = np.array([
        [1395.4279454761115, 369.19738186844506], [990.9949418465998, 310.0160005089381],
        [925.4665129351163, 707.1974150372425], [1318.0178816950363, 793.6315168459968],
        [1112.7634529938568, 549.0694665378237], [1595.5075621851815, 210.5603243516652],
        [849.2583326747649, 121.0281652735058], [736.3876406231022, 832.1544609141527],
        [1445.6001343209805, 1006.7198598891247], [1039.9635888876967, 554.0864755847832],
    ])

    image_points_rosys = [
        Point(x=image_points[x, 0], y=image_points[x, 1]) for x in range(image_points.shape[0])
    ]

    camera_matrix = np.array([
        [1066.7203851114377, 0.0, 962.498477879789],
        [0.0, 1069.03584263432, 533.5056801829332],
        [0.0, 0.0, 1.0],
    ])

    distortion = np.array([
        1.8027944406871372, 0.09296596486660615,
        -0.0009082271491742374, -0.0001919076200299327,
        -0.020278074186048444, 2.24370640981634,
        0.7354262632422073, -0.06533877394712581
    ])

    intrinsics = Intrinsics(matrix=camera_matrix.tolist(),
                            distortion=distortion.tolist(),
                            size=ImageSize(width=1920, height=1080))

    p0 = Point3d(x=500, y=0, z=3500)
    r0 = Rotation.from_euler(np.pi+0.2, np.pi/5, 0)
    s0 = np.linspace(0, 1, len(world_lines))

    sr = SpatialResection(intrinsics)

    if num_world_points == 0:
        result = sr.lsa_with_lines(
            p0=p0, r0=r0, s0=s0 if use_numpy else list(s0),
            world_lines=world_lines if use_numpy else world_lines_rosys,
            image_line_points=image_points if use_numpy else image_points_rosys)
    else:
        n_l = num_world_points
        result = sr.lsa_with_lines(
            p0=p0, r0=r0, s0=None,
            world_lines=world_lines[n_l:] if use_numpy else world_lines_rosys[n_l:],
            image_line_points=image_points[n_l:] if use_numpy else image_points_rosys[n_l:],
            world_points=world_points[:n_l] if use_numpy else world_points_rosys[:n_l],
            image_points=image_points[:n_l] if use_numpy else image_points_rosys[:n_l])

    camera_pose = result.camera_pose

    # Check the camera pose
    ground_truth_translation = Point3d(x=400, y=100, z=2450)
    ground_truth_rotation = Rotation.from_euler(np.pi, 20/180*np.pi, 10/180*np.pi)

    assert np.allclose(camera_pose.point_3d.array, ground_truth_translation.array, atol=2)
    assert np.allclose(camera_pose.rotation.quaternion, ground_truth_rotation.quaternion, atol=0.001)

    # Check the estimated object space points on lines
    for line_point, world_point in zip(result.estimated_points_on_lines or [], world_points_rosys[num_world_points:], strict=False):
        assert np.allclose(line_point.array, world_point.array, atol=5)
