from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import overload

import cv2
import numpy as np

from ..geometry import Point, Point3d, Rotation
from .image import Image, ImageSize


class CameraModel(str, Enum):
    PINHOLE = 'pinhole'
    FISHEYE = 'fisheye'
    OMNIDIRECTIONAL = 'omnidirectional'


@dataclass(slots=True, kw_only=True)
class Intrinsics:
    """The intrinsic parameters of a camera.

    :param model: The camera model to use.
    :param matrix: The camera matrix K.
    :param distortion: The distortion coefficients D.
    :param xi: The omnidirectional camera parameter xi (only for ``CameraModel.OMNIDIRECTIONAL``).
    :param rotation: An inner rotation matrix, useful for visual-inertial calibration or omnidirectional projection.
    :param size: The size of the image.
    """
    model: CameraModel = CameraModel.PINHOLE
    matrix: list[list[float]]
    distortion: list[float]
    xi: float = 0.0
    rotation: Rotation = field(default_factory=Rotation.zero)
    size: ImageSize

    @staticmethod
    def create_default(width: int = 800, height: int = 600, *, focal_length: float = 570) -> Intrinsics:
        size = ImageSize(width=width, height=height)
        K: list[list[float]] = [[focal_length, 0, size.width / 2],
                                [0, focal_length, size.height / 2],
                                [0, 0, 1]]
        D: list[float] = [0] * 5
        rotation = Rotation.zero()
        return Intrinsics(matrix=K, distortion=D, rotation=rotation, size=size)


log = logging.getLogger('rosys.world.calibration')


@dataclass(slots=True, kw_only=True)
class Extrinsics:
    """The extrinsic parameters of a camera.

    :param rotation: The rotation matrix R.
    :param translation: The translation vector t.
    """
    rotation: Rotation = field(default_factory=lambda: Rotation.from_euler(np.pi, 0, 0))
    translation: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])


@dataclass(slots=True, kw_only=True)
class Calibration:
    """Represents the full calibration of a camera."""
    intrinsics: Intrinsics
    extrinsics: Extrinsics = field(default_factory=Extrinsics)

    @property
    def rotation(self) -> Rotation:
        return self.extrinsics.rotation * self.intrinsics.rotation

    @property
    def rotation_array(self) -> np.ndarray:
        return np.dot(self.extrinsics.rotation.R, self.intrinsics.rotation.R)

    @property
    def center_point(self) -> Point3d:
        return Point3d(x=self.extrinsics.translation[0], y=self.extrinsics.translation[1], z=self.extrinsics.translation[2])

    @staticmethod
    def from_points(world_points: list[Point3d] | list[list[Point3d]],
                    image_points: list[Point] | list[list[Point]],
                    image_size: ImageSize,
                    f0: float,
                    rational_model: bool = False,
                    camera_model: CameraModel = CameraModel.PINHOLE) -> Calibration:
        """Estimate the camera calibration from corresponding world and image points.

        :param world_points: The observed points in 3D world coordinates.
        :param image_points: The observed points in 2D image coordinates.
        :param image_size: The size of the image.
        :param f0: An initial guess for the focal length.
        :param rational_model: Whether to use the rational camera model (only applies to pinhole cameras).
        :param camera_model: The camera model to use.

        :return: The estimated camera calibration.
        """
        if rational_model and camera_model != CameraModel.PINHOLE:
            raise ValueError('Rational model is only supported for pinhole cameras')

        if isinstance(world_points[0], Point3d) or isinstance(image_points[0], Point):
            assert len(world_points) == len(image_points), 'Image and world points are not formatted equally'
            world_points = [world_points]  # type: ignore
            image_points = [image_points]  # type: ignore

        world_point_array = [np.array([p.tuple for p in view], dtype=np.float32).reshape((-1, 1, 3))  # type: ignore
                             for view in world_points]
        image_point_array = [np.array([p.tuple for p in view], dtype=np.float32).reshape((-1, 1, 2))  # type: ignore
                             for view in image_points]
        K0 = np.array([[f0, 0, image_size.width / 2], [0, f0, image_size.height / 2], [0, 0, 1]], dtype=np.float32)
        D0 = np.array([0.1, 0.4, -0.5, 0.2], dtype=np.float32).reshape(1, 4)

        xi: float = 0.0

        optimization_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-6)

        if camera_model == CameraModel.PINHOLE:
            flags = cv2.CALIB_USE_INTRINSIC_GUESS
            if rational_model:
                flags |= cv2.CALIB_RATIONAL_MODEL
            _rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
                objectPoints=world_point_array,
                imagePoints=image_point_array,
                imageSize=image_size.tuple,
                cameraMatrix=K0,
                distCoeffs=np.zeros((1, 5), dtype=np.float32),
                flags=flags,
                criteria=optimization_criteria,
            )
        elif camera_model == CameraModel.FISHEYE:
            flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
            flags |= cv2.fisheye.CALIB_CHECK_COND
            flags |= cv2.fisheye.CALIB_FIX_SKEW
            flags |= cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
            _rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(  # pylint: disable=unpacking-non-sequence
                objectPoints=world_point_array,
                imagePoints=image_point_array,
                image_size=image_size.tuple,
                K=K0,
                D=D0,
                flags=flags,
                criteria=optimization_criteria,
            )
            if D.size != 4:
                raise ValueError(f'Fisheye calibration failed (got invalid distortion coefficients D="{D}")')
        elif camera_model == CameraModel.OMNIDIRECTIONAL:
            flags = cv2.omnidir.CALIB_USE_GUESS
            flags |= cv2.omnidir.CALIB_FIX_SKEW
            _rms, K, xi_arr, D, rvecs, tvecs, _status = cv2.omnidir.calibrate(  # pylint: disable=unpacking-non-sequence
                objectPoints=world_point_array,
                imagePoints=image_point_array,
                size=image_size.tuple,
                K=K0,
                xi=np.array([xi], dtype=np.float32),
                D=D0,
                flags=flags,
                criteria=optimization_criteria,
            )
            xi = xi_arr[0]
            if D.size != 4:
                raise ValueError(f'Omnidirectional calibration failed (got invalid distortion coefficients D="{D}")')
        else:
            raise ValueError(f'Unknown camera model "{camera_model}"')

        rotation0 = Rotation(R=np.eye(3).tolist())
        intrinsics = Intrinsics(matrix=K.tolist(),
                                distortion=D.tolist()[0],
                                rotation=rotation0,
                                size=image_size,
                                model=camera_model,
                                xi=xi)

        rotation = Rotation.from_rvec(rvecs[0]).T
        translation = (-np.array(rotation.R).dot(tvecs[0])).flatten().tolist()
        extrinsics = Extrinsics(rotation=rotation, translation=translation)

        return Calibration(intrinsics=intrinsics, extrinsics=extrinsics)

    @overload
    def project_to_image(self, world_coordinates: Point3d) -> Point | None: ...

    @overload
    def project_to_image(self, world_coordinates: np.ndarray) -> np.ndarray: ...

    def project_to_image(self, world_coordinates: Point3d | np.ndarray) -> Point | None | np.ndarray:
        """Project a point in world coordinates to the image plane.

        This takes into account the camera's intrinsic and extrinsic parameters.
        """
        if isinstance(world_coordinates, Point3d):
            world_array = np.array([world_coordinates.tuple], dtype=np.float32)
            image_array = self.project_to_image(world_array)
            if np.isnan(image_array).any():
                return None
            return Point(x=image_array[0, 0], y=image_array[0, 1])  # pylint: disable=unsubscriptable-object

        R = self.rotation_array
        Rod = cv2.Rodrigues(R.T)[0]
        t = -R.T @ self.extrinsics.translation
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion, dtype=float)

        # pylint: disable=unpacking-non-sequence
        if self.intrinsics.model == CameraModel.PINHOLE:
            image_array, _ = cv2.projectPoints(world_coordinates, Rod, t, K, D)
        elif self.intrinsics.model == CameraModel.FISHEYE:
            image_array, _ = cv2.fisheye.projectPoints(world_coordinates.reshape(-1, 1, 3), Rod, t, K, D)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            xi = self.intrinsics.xi
            image_array, _ = cv2.omnidir.projectPoints(world_coordinates.reshape(-1, 1, 3), Rod, t, K, xi, D)
        else:
            raise ValueError(f'Unknown camera model "{self.intrinsics.model}"')
        # pylint: enable=unpacking-non-sequence

        if self.intrinsics.model != CameraModel.OMNIDIRECTIONAL:
            world_coordinates = world_coordinates.reshape(-1, 3)
            local_coordinates = (world_coordinates - np.array(self.extrinsics.translation)) @ R
            image_array[local_coordinates[:, 2] < 0, :] = np.nan

        return image_array.reshape(-1, 2)

    @overload
    def project_from_image(self, image_coordinates: Point, target_height: float = 0) -> Point3d | None: ...

    @overload
    def project_from_image(self, image_coordinates: np.ndarray, target_height: float = 0) -> np.ndarray: ...

    def project_from_image(self, image_coordinates: Point | np.ndarray, target_height: float = 0) -> Point3d | None | np.ndarray:
        """Project a point in image coordinates to a plane in world xy dimensions at a given height.

        :param image_coordinates: The image coordinates to project.
        :param target_height: The height of the plane in world coordinates.

        :return: The world coordinates of the projected point.
        """
        if isinstance(image_coordinates, Point):
            image_points = np.array(image_coordinates.tuple, dtype=np.float32)
            world_points = self.project_from_image(image_points, target_height=target_height)
            if np.isnan(world_points).any():
                return None
            return Point3d(x=world_points[0, 0], y=world_points[0, 1], z=world_points[0, 2])  # pylint: disable=unsubscriptable-object

        image_rays = self.points_to_rays(image_coordinates.astype(np.float32).reshape(-1, 1, 2))
        objPoints = image_rays @ self.rotation_array.T
        Z = self.extrinsics.translation[-1]
        t = np.array(self.extrinsics.translation)
        world_points = t.T - objPoints * (Z - target_height) / objPoints[:, 2:]

        reprojection = self.project_to_image(world_points)
        sign = objPoints[:, -1] * np.sign(Z)
        distance = np.linalg.norm(reprojection - image_coordinates.reshape(-1, 2), axis=1)
        world_points[np.logical_not(np.logical_and(sign < 0, distance < 2)), :] = np.nan

        return world_points

    def points_to_rays(self, image_points: np.ndarray) -> np.ndarray:
        """Convert image points to rays in homogeneous coordinates with respect to the camera coordinate frame."""
        K = np.array(self.intrinsics.matrix, dtype=np.float32).reshape((3, 3))
        D = np.array(self.intrinsics.distortion)
        if self.intrinsics.model == CameraModel.PINHOLE:
            undistorted = cv2.undistortPoints(image_points, K, D)
        elif self.intrinsics.model == CameraModel.FISHEYE:
            undistorted = cv2.fisheye.undistortPoints(image_points, K, D)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            R = np.array(self.intrinsics.rotation.R, dtype=np.float32)
            xi = np.array(self.intrinsics.xi, dtype=np.float32)
            undistorted = cv2.omnidir.undistortPoints(image_points, K, D, xi=xi, R=R)
        else:
            raise ValueError(f'Unknown camera model "{self.intrinsics.model}"')

        return cv2.convertPointsToHomogeneous(undistorted).reshape(-1, 3)

    def undistort_points(self, image_points: np.ndarray, crop: bool = False) -> np.ndarray:
        """Generalized wrapper for undistorting image points.

        :param image_points: The image points to undistort.
        :param crop: Whether cropping is applied to the image during undistortion.

        :return: The undistorted image points.
        """
        K = np.array(self.intrinsics.matrix, dtype=np.float32).reshape((3, 3))
        D = np.array(self.intrinsics.distortion)
        if self.intrinsics.model == CameraModel.PINHOLE:
            new_K = self.get_undistorted_camera_matrix(crop=crop)
            return cv2.undistortPoints(image_points, K, D, P=new_K, R=np.eye(3))
        elif self.intrinsics.model == CameraModel.FISHEYE:
            new_K = self.get_undistorted_camera_matrix(crop=crop)
            return cv2.fisheye.undistortPoints(image_points, K, D, P=new_K)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            R = np.array(self.intrinsics.rotation.R, dtype=np.float32)
            xi = np.array(self.intrinsics.xi, dtype=np.float32)
            return cv2.omnidir.undistortPoints(image_points, K, D, xi=xi, R=R)
        else:
            raise ValueError(f'Unknown camera model "{self.intrinsics.model}"')

    def distort_points(self, image_points: np.ndarray, crop: bool = False) -> np.ndarray:
        """Generalized wrapper for distorting image points.

        :param image_points: The image points to distort.
        :param crop: Whether cropping is applied to the image during distortion.

        :return: The distorted image points.
        """
        K = np.array(self.intrinsics.matrix, dtype=np.float32).reshape((3, 3))
        D = np.array(self.intrinsics.distortion)

        if self.intrinsics.model == CameraModel.PINHOLE:
            new_K = self.get_undistorted_camera_matrix(crop=crop)
            return cv2.undistortPoints(image_points, K, D, P=new_K, R=np.eye(3))
        elif self.intrinsics.model == CameraModel.FISHEYE:
            new_K = self.get_undistorted_camera_matrix(crop=crop)
            normalized_points = np.linalg.inv(new_K) @ cv2.convertPointsToHomogeneous(image_points).reshape(-1, 3).T
            normalized_points = cv2.convertPointsFromHomogeneous(normalized_points.T).reshape(-1, 1, 2)
            return cv2.fisheye.distortPoints(normalized_points, K, D)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            raise NotImplementedError('Re-distortion for omnidirectional cameras is not supported')
        else:
            raise ValueError(f'Unknown camera model "{self.intrinsics.model}"')

    def get_undistorted_camera_matrix(self, crop: bool = False) -> np.ndarray:
        """Compute the camera matrix for the undistorted image.

        :param crop: Whether cropping is applied to the image during undistortion.

        :return: The camera matrix for the undistorted image.
        """
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        w, h = self.intrinsics.size.width, self.intrinsics.size.height
        if self.intrinsics.model == CameraModel.PINHOLE:
            new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h), centerPrincipalPoint=True)
        elif self.intrinsics.model == CameraModel.FISHEYE:
            new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3),
                                                                           balance=0.0 if crop else 1.0,
                                                                           new_size=(w, h),
                                                                           fov_scale=1)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            if crop:
                logging.warning('Cropping is not yet supported for omnidirectional cameras')
            new_K = np.array([[w / 4, 0, w / 2],
                              [0, h / 4, h / 2],
                              [0, 0, 1]])
        else:
            raise ValueError(f'Unknown camera model "{self.intrinsics.model}"')

        return new_K

    def undistort_array(self, image_array: np.ndarray, crop: bool = False) -> np.ndarray:
        """Undistort an image represented as a numpy array.

        The image is expected to be decoded (in particular not encoded bytes of a JPEG image).

        :param image_array: The image to undistort.
        :param crop: Whether cropping is applied to the image during undistortion.

        :return: The undistorted image.
        """
        if image_array.shape[0] != self.intrinsics.size.height or image_array.shape[1] != self.intrinsics.size.width:
            log.warning('Image size does not match calibration size (image: %s, calibration: %s)',
                        image_array.shape, self.intrinsics.size)
            return image_array

        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        h, w = image_array.shape[:2]
        if self.intrinsics.model == CameraModel.PINHOLE:
            new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h), centerPrincipalPoint=True)

            dst = cv2.undistort(image_array, K, D, None, new_K)
            if crop:
                x, y, roi_w, roi_h = roi
                dst = dst[y:y+roi_h, x:x+roi_w]
        elif self.intrinsics.model == CameraModel.FISHEYE:
            balance = 0.0 if crop else 1.0
            new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=balance)

            map1, map2 = cv2.fisheye.initUndistortRectifyMap(  # pylint: disable=unpacking-non-sequence
                K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)
            dst = cv2.remap(image_array, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            flags = cv2.omnidir.RECTIFY_PERSPECTIVE
            new_K = self.get_undistorted_camera_matrix(crop=crop)

            R = np.array(self.intrinsics.rotation.R)
            xi = np.array(self.intrinsics.xi)
            dst = cv2.omnidir.undistortImage(image_array, K=K, D=D, xi=xi, Knew=new_K,
                                             flags=flags, new_size=(w, h), R=R)
        else:
            raise ValueError(f'Unknown camera model "{self.intrinsics.model}"')

        return dst

    def undistort_image(self, image: Image) -> Image:
        """Undistort an image represented as an Image object.

        If you already have the image as an unencoded numpy array, use ``undistort_array`` instead.

        :param image: The image to undistort.

        :return: The undistorted image.
        """
        return Image(
            camera_id=image.camera_id,
            size=image.size,
            time=image.time,
            data=cv2.imencode('.jpg', self.undistort_array(image.to_array()))[1].tobytes(),
            is_broken=image.is_broken,
            tags=image.tags,
        )

    def get_undistorted_size(self) -> ImageSize:
        """Compute the size of the undistorted image after cropping.

        :return: The size of the undistorted image.
        """
        if self.intrinsics.model == CameraModel.PINHOLE:
            K = np.array(self.intrinsics.matrix)
            D = np.array(self.intrinsics.distortion)
            h, w = self.intrinsics.size.height, self.intrinsics.size.width
            _, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h), centerPrincipalPoint=True)
            return ImageSize(width=roi[2], height=roi[3])
        else:
            return self.intrinsics.size
