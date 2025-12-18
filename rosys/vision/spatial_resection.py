from dataclasses import dataclass

import cv2
import numpy as np
from scipy.optimize import least_squares

from rosys.geometry import Point3d, Pose3d, Rotation
from rosys.vision.calibration import Calibration, Intrinsics


@dataclass(slots=True, kw_only=True)
class SpatialResectionResult:
    """Result of the spatial resection with lines."""
    success: bool
    iterations: int
    average_reprojection_error: float
    camera_pose: Pose3d
    running_variables: list[float]
    estimated_points_on_lines: list[Point3d]


class SpatialResection:

    def __init__(self, intrinsics: Intrinsics) -> None:
        """Spatial resection to find a camera pose from correspondences between object space and image space."""
        self.intrinsics = intrinsics

    def pnp_with_lsa(self,
                     *,
                     world_points: np.ndarray,
                     image_points: np.ndarray,
                     algorithm: int | str | None = None,
                     p0: Point3d | None = None,
                     r0: Rotation | None = None) -> SpatialResectionResult:
        """Solve the spatial resection problem using OpenCV's PnP and finalize with a least-squares refinement.

        :param world_points: The 3D coordinates of the points in object space, shape (n, 3)
        :param image_points: The 2D coordinates of the points in the image, shape (n, 2)
        :param algorithm: PnP algorithm flag or name (e.g., 'ITERATIVE', 'EPNP', 'P3P', 'AP3P', 'IPPE'). If None, choose automatically.
        :param p0: Initial position of the camera (optional)
        :param r0: Initial rotation of the camera (optional)

        :return: The result of the spatial resection
        """
        assert world_points.shape[0] == image_points.shape[0] and world_points.shape[0] >= 3, 'Need at least 3 correspondences'

        if np.isnan(world_points).any() or np.isnan(image_points).any():
            raise ValueError('Points contain NaNs')

        # Prepare calibration and undistort image observations for a unified pinhole model
        calibration = Calibration(intrinsics=self.intrinsics)
        K_undist = calibration.get_undistorted_camera_matrix()

        object_points: np.ndarray = world_points.astype(np.float64).reshape(-1, 1, 3)
        image_points_undist = calibration.undistort_points(image_points.astype(np.float64).reshape(-1, 1, 2))
        D_zeros: np.ndarray = np.zeros((1, 5), dtype=np.float64)

        # Decide on algorithm
        def is_planar(points: np.ndarray) -> bool:
            centered = points.reshape(-1, 3) - points.reshape(-1, 3).mean(axis=0)
            _, s, _ = np.linalg.svd(centered, full_matrices=False)
            return s[-1] / max(s[0], 1e-12) < 1e-3

        if algorithm is None:
            # Automatic selection
            num_points = object_points.shape[0]
            if num_points >= 4 and is_planar(object_points):
                method_flag = cv2.SOLVEPNP_IPPE
            elif num_points >= 6:
                method_flag = cv2.SOLVEPNP_EPNP
            elif num_points == 3:
                method_flag = cv2.SOLVEPNP_P3P
            elif num_points >= 4:
                method_flag = cv2.SOLVEPNP_AP3P
            else:
                method_flag = cv2.SOLVEPNP_ITERATIVE
        elif isinstance(algorithm,  int):
            method_flag = algorithm
        elif isinstance(algorithm, str):
            name = algorithm.upper()
            if name == 'ITERATIVE':
                method_flag = cv2.SOLVEPNP_ITERATIVE
            elif name == 'EPNP':
                method_flag = cv2.SOLVEPNP_EPNP
            elif name == 'P3P':
                method_flag = cv2.SOLVEPNP_P3P
            elif name == 'AP3P':
                method_flag = cv2.SOLVEPNP_AP3P
            elif name == 'IPPE':
                method_flag = cv2.SOLVEPNP_IPPE
            else:
                raise ValueError(f'Unknown PnP algorithm: {algorithm}')
        else:
            raise TypeError('Algorithm must be int, str or None')

        # Run PnP on undistorted points with zero distortion and K_undist
        use_guess = p0 is not None and r0 is not None
        if use_guess:
            assert p0 is not None and r0 is not None
            rvec_init = cv2.Rodrigues(r0.T.matrix)[0]
            tvec_init = -r0.T.matrix @ p0.array.reshape(3, 1)
        else:
            rvec_init = None
            tvec_init = None

        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points_undist, K_undist, D_zeros,
            rvec_init, tvec_init, use_guess, int(method_flag)
        )
        if not ok:
            # Fallback to ITERATIVE
            ok, rvec, tvec = cv2.solvePnP(
                object_points, image_points_undist, K_undist, D_zeros,
                rvec_init, tvec_init, use_guess, int(cv2.SOLVEPNP_ITERATIVE)
            )
        if not ok:
            return SpatialResectionResult(
                success=False,
                iterations=0,
                average_reprojection_error=float('nan'),
                camera_pose=Pose3d(x=float('nan'), y=float('nan'), z=float('nan'), rotation=Rotation.zero()),
                running_variables=[],
                estimated_points_on_lines=[],
            )

        cv2.solvePnPRefineLM(object_points, image_points_undist, K_undist, D_zeros, rvec, tvec)

        Rwc = Rotation.from_rvec(rvec).T
        # Ensure types are numpy arrays so the `@` operator is type-safe for static checkers
        rmat = np.asarray(cv2.Rodrigues(rvec)[0], dtype=np.float64)
        tvec_arr = np.asarray(tvec, dtype=np.float64).reshape(3)
        C = (-rmat.T @ tvec_arr).reshape(3)

        # Compute reprojection error on all observations (undistorted domain)
        proj_all, _ = cv2.projectPoints(object_points, rvec, tvec, K_undist, D_zeros)
        proj_all_np = np.asarray(proj_all, dtype=np.float64).reshape(-1, 2)
        img_all_np = image_points_undist.reshape(-1, 2)
        residuals_all = proj_all_np - img_all_np
        avg_reproj_error = float(np.mean(np.abs(residuals_all)))

        return SpatialResectionResult(
            success=True,
            iterations=1,
            average_reprojection_error=avg_reproj_error,
            camera_pose=Pose3d(x=float(C[0]), y=float(C[1]), z=float(C[2]), rotation=Rwc),
            running_variables=[],
            estimated_points_on_lines=[],
        )

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
        :raises ValueError: If the Residuals are not finite in the initial state
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
        ])

        # Prepare state vector
        # - 3D position of the camera
        # - 4D quaternion of the camera rotation
        # - m running variables for the lines
        # => [x, y, z, q_w, q_x, q_y, q_z, s_0, s_1, ..., s_m]
        x_0 = np.concatenate([
            p0.array.ravel(),
            r0.T.quaternion,
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

            # Project 3D points to 2D image points
            Xc = all_world_points @ R.T
            image_points_projected[:] = (Xc @ camera_matrix.T)[:, :2] / Xc[:, 2:3]

            # Compute residuals
            np.subtract(image_points_projected, all_image_points, out=residuals)
            return residuals.flatten()

        res = least_squares(f, x_0, **ls_args)

        return SpatialResectionResult(
            success=res.success,
            iterations=res.nfev,
            average_reprojection_error=float(np.mean(np.abs(res.fun))),
            camera_pose=Pose3d(x=res.x[0], y=res.x[1], z=res.x[2], rotation=Rotation.from_quaternion(*res.x[3:7]).T),
            running_variables=res.x[7:].tolist(),
            estimated_points_on_lines=[
                Point3d(x=x, y=y, z=z)
                for x, y, z in world_lines[:, :3] + world_lines[:, 3:] * res.x[7:, None]
            ],
        )
