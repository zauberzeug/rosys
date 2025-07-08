from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from rosys.geometry import Point3d, Pose3d, Rotation
from rosys.vision.calibration import Calibration, Intrinsics


@dataclass(slots=True, kw_only=True)
class SpatialResectionResult:
    """Result of the spatial resection."""
    success: bool
    iterations: int
    average_reprojection_error: float
    camera_pose: Pose3d
    running_variables: list[float] | None  # None?
    estimated_points_on_lines: list[Point3d] | None  # None?


class SpatialResection:

    def __init__(self, intrinsics: Intrinsics) -> None:
        """Spatial resection to find a camera pose from correspondences between object space and image space."""
        self.intrinsics = intrinsics

    def lsa_with_lines(self,
                       *,
                       p0: Point3d,
                       r0: Rotation,
                       world_lines: np.ndarray,
                       image_line_points: np.ndarray,
                       s0: np.ndarray | None = None,
                       world_points: np.ndarray | None = None,
                       image_points: np.ndarray | None = None,
                       **ls_args) -> SpatialResectionResult:
        """Solve the spatial resection problem by minimizing the reprojection error.

        The problem is solved using a least squares adjustment to minimize the reprojection error between:

        a) 3D lines and observed 2D points on the lines
        b) 3D points and their 2D projections

        :param p0: The initial position of the camera
        :param r0: The initial rotation of the camera
        :param world_lines: The lines in object space (origin, direction), shape (m, 6)
        :param image_line_points: The 2D coordinates of the points in the image, shape (m, 2)
        :param s0: The initial running variables of the lines, shape (m,) or None
        :param world_points: The 3D coordinates of the points in object space, shape (n, 3) or None
        :param image_points: The 2D coordinates of the points in the image, shape (n, 2) or None
        :param ls_args: Additional arguments for the least squares solver

        :return: The result of the spatial resection
        """
        # Prepare inputs
        if world_points is None:
            world_points = np.zeros((0, 3))
        if image_points is None:
            image_points = np.zeros((0, 2))
        if s0 is None:
            s0 = np.zeros(len(world_lines))

        # Check sizes
        assert len(world_lines) == len(image_line_points)
        m = len(world_lines)
        assert len(world_points) == len(image_points)
        n = len(world_points)

        # Undistort observations
        calibration = Calibration(intrinsics=self.intrinsics)
        camera_matrix = calibration.get_undistorted_camera_matrix()
        all_image_points = np.concatenate([
            calibration.undistort_points(image_points),
            calibration.undistort_points(image_line_points),
        ], axis=0)

        # Prepare state vector
        # - 3D position of the camera
        # - 4D quaternion of the camera rotation
        # - m running variables for the lines
        # => [x, y, z, q_w, q_x, q_y, q_z, s_0, s_1, ..., s_m]
        x_0 = np.concatenate([
            p0.array.ravel(),
            r0.quaternion,
            s0,
        ])

        # Memory allocation (to avoid reallocations)
        R = np.eye(3).astype(np.float64)
        all_world_points = np.zeros(shape=(n + m, 3))
        image_points_projected = np.zeros(shape=(n + m, 2))
        residuals = np.zeros(shape=(n + m, 2))
        world_line_points = np.zeros(shape=(m, 3))

        def _quaternion_to_rotation_matrix(q: np.ndarray, R: np.ndarray) -> None:
            """Inplace conversion of quaternion to rotation matrix.

            :param q: The quaternion to convert to a rotation matrix: [w, x, y, z]
            :param R: The rotation matrix to fill
            """
            q /= np.linalg.norm(q)
            R[0, 0] = 1 - 2 * q[2]**2 - 2 * q[3]**2
            R[0, 1] = 2 * q[1] * q[2] - 2 * q[0] * q[3]
            R[0, 2] = 2 * q[1] * q[3] + 2 * q[0] * q[2]
            R[1, 0] = 2 * q[1] * q[2] + 2 * q[0] * q[3]
            R[1, 1] = 1 - 2 * q[1]**2 - 2 * q[3]**2
            R[1, 2] = 2 * q[2] * q[3] - 2 * q[0] * q[1]
            R[2, 0] = 2 * q[1] * q[3] - 2 * q[0] * q[2]
            R[2, 1] = 2 * q[2] * q[3] + 2 * q[0] * q[1]
            R[2, 2] = 1 - 2 * q[1]**2 - 2 * q[2]**2

        def f(x: np.ndarray) -> np.ndarray:
            """Functional model using the collinearity equations.

            :param x: The state vector: [x, y, z, q_w, q_x, q_y, q_z, s_0, s_1, ..., s_m]
            :return: The residuals
            """
            # Transform Object coordinates to Camera coordinates
            C = x[:3]
            q = x[3:7]
            _quaternion_to_rotation_matrix(q, R)

            np.multiply(world_lines[:, 3:], x[7:].reshape(-1, 1), out=world_line_points)
            np.add(world_line_points, world_lines[:, :3], out=world_line_points)

            all_world_points[:n] = world_points
            all_world_points[n:] = world_line_points
            np.subtract(all_world_points, C, out=all_world_points)

            Xc = all_world_points @ R.T

            # Project 3D points to 2D image points
            image_points_projected[:, 0] = camera_matrix[0, 0] * Xc[:, 0] / Xc[:, 2] + camera_matrix[0, 2]
            image_points_projected[:, 1] = camera_matrix[1, 1] * Xc[:, 1] / Xc[:, 2] + camera_matrix[1, 2]
            # image_points_projected[:] = (Xc @ camera_matrix.T)[:, :2] / Xc[:, 2:3]

            # Compute residuals
            np.subtract(image_points_projected, all_image_points, out=residuals)
            return residuals.flatten()

        res = least_squares(f, x_0, **ls_args)

        return SpatialResectionResult(
            success=res.success,
            iterations=res.nfev,
            average_reprojection_error=np.mean(np.abs(res.fun)),
            camera_pose=Pose3d(x=res.x[0], y=res.x[1], z=res.x[2], rotation=Rotation.from_quaternion(*res.x[3:7]).T),
            running_variables=res.x[7:],
            estimated_points_on_lines=[
                Point3d(x=x, y=y, z=z)
                for x, y, z in world_lines[:, :3] + world_lines[:, 3:] * res.x[7:, None]
            ],
        )
