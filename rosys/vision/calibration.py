from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import cast, overload

import cv2
import numpy as np
from numpy.typing import NDArray

from ..geometry import Frame3d, Point, Point3d, Pose3d, Rotation
from ..helpers.deprecation import deprecated_function
from .image import Image, ImageSize

FloatArray = NDArray[np.float32] | NDArray[np.float64]


class CameraModel(str, Enum):
    PINHOLE = 'pinhole'
    FISHEYE = 'fisheye'
    OMNIDIRECTIONAL = 'omnidirectional'


@dataclass(slots=True, kw_only=True)
class OmnidirParameters:
    """The parameters for an omnidirectional camera.

    :param xi: The omnidirectional camera parameter xi.
    :param rotation: Rotation used to determine crop when undistorting to pinhole image.
    """
    xi: float = 0.0
    rotation: Rotation = field(default_factory=Rotation.zero)


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
    omnidir_params: OmnidirParameters | None = None
    size: ImageSize

    @staticmethod
    def create_default(width: int = 800, height: int = 600, *, focal_length: float = 570) -> Intrinsics:
        size = ImageSize(width=width, height=height)
        K: list[list[float]] = [[focal_length, 0, size.width / 2],
                                [0, focal_length, size.height / 2],
                                [0, 0, 1]]
        D: list[float] = [0] * 5
        return Intrinsics(matrix=K, distortion=D, size=size)


log = logging.getLogger('rosys.vision.calibration')


@dataclass(slots=True, kw_only=True)
class Calibration:
    """Represents the full calibration of a camera."""
    intrinsics: Intrinsics
    extrinsics: Pose3d = field(default_factory=Pose3d)

    @staticmethod
    def from_points(world_points: list[Point3d] | list[list[Point3d]],
                    image_points: list[Point] | list[list[Point]],
                    *,
                    image_size: ImageSize,
                    f0: float,
                    rational_model: bool = False,
                    camera_model: CameraModel = CameraModel.PINHOLE,
                    frame: Frame3d | None = None) -> Calibration:
        """Estimate the camera calibration from corresponding world and image points.

        :param world_points: The observed points in 3D world coordinates.
        :param image_points: The observed points in 2D image coordinates.
        :param image_size: The size of the image.
        :param f0: An initial guess for the focal length.
        :param rational_model: Whether to use the rational camera model (only applies to pinhole cameras).
        :param camera_model: The camera model to use.
        :param frame: The coordinate frame of the world points and the extrinsic camera pose.

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

        intrinsics = Intrinsics(matrix=K.tolist(),
                                distortion=D.tolist()[0],
                                size=image_size,
                                model=camera_model,
                                omnidir_params=OmnidirParameters(xi=xi))

        rotation = Rotation.from_rvec(rvecs[0]).T
        translation = -np.array(rotation.R).dot(tvecs[0])
        extrinsics = Pose3d(x=translation[0, 0],
                            y=translation[1, 0],
                            z=translation[2, 0],
                            rotation=rotation).in_frame(frame)

        return Calibration(intrinsics=intrinsics, extrinsics=extrinsics)

    @overload
    def project_to_image(self, coordinates: Point3d) -> Point | None:
        """Project a 3D point to the image plane.

        :param coordinates: The 3D point to project.
        :return: The image coordinates of the projected point, or None if projection failed.
        """

    @overload
    def project_to_image(self, coordinates: list[Point3d]) -> list[Point | None]:
        """Project 3D points to the image plane.

        :param coordinates: The list of 3D points to project.
        :return: The image coordinates of the projected points, or None if any projection failed.
        """

    @overload
    def project_to_image(self, coordinates: FloatArray, *, frame: Frame3d | None = None) -> FloatArray:
        """Project 3D points to the image plane.

        :param coordinates: (Nx3) The 3D coordinates of the points to project.
        :param frame: The coordinate frame of the coordinates.
        :return: (Nx2) The image coordinates of the projected points.
        """

    def project_to_image(self, coordinates: Point3d | list[Point3d] | FloatArray, *, frame: Frame3d | None = None) -> Point | None | list[Point | None] | FloatArray:

        if isinstance(coordinates, Point3d):
            return self.project_to_image([coordinates])[0]
        if isinstance(coordinates, list):
            world_array = np.array([point.resolve().tuple for point in coordinates], dtype=np.float64)
            image_array = self.project_to_image(world_array)
            return [Point(x=vector[0], y=vector[1]) if not np.isnan(vector).any() else None for vector in image_array]

        world_extrinsics = self.extrinsics.relative_to(frame)
        R = world_extrinsics.rotation.matrix.astype(np.float64)
        Rod = cv2.Rodrigues(R.T)[0]
        t = -R.T @ world_extrinsics.translation
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion, dtype=float)

        # pylint: disable=unpacking-non-sequence
        if self.intrinsics.model == CameraModel.PINHOLE:
            image_array, _ = cast(FloatArray, cv2.projectPoints(coordinates, Rod, t, K, D))
        elif self.intrinsics.model == CameraModel.FISHEYE:
            image_array, _ = cast(FloatArray, cv2.fisheye.projectPoints(coordinates.reshape(-1, 1, 3), Rod, t, K, D))
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            assert self.intrinsics.omnidir_params is not None, 'Omnidirectional parameters are unset'
            xi = self.intrinsics.omnidir_params.xi
            image_array, _ = cast(FloatArray, cv2.omnidir.projectPoints(
                coordinates.reshape(-1, 1, 3), Rod, t, K, xi, D))
        else:
            raise ValueError(f'Unknown camera model "{self.intrinsics.model}"')
        # pylint: enable=unpacking-non-sequence

        if self.intrinsics.model != CameraModel.OMNIDIRECTIONAL:
            coordinates = coordinates.reshape(-1, 3)
            local_coordinates = (coordinates - world_extrinsics.translation) @ R
            image_array[local_coordinates[:, 2] < 0, :] = np.nan

        return image_array.reshape(-1, 2)

    @overload
    def project_from_image(self, image_coordinates: Point, target_height: float = 0) -> Point3d | None:
        """Project a point in image coordinates to a plane in world coordinates.

        :param image_coordinates: The image coordinates to project.
        :param target_height: The height of the plane in world coordinates.
        :return: The world coordinates of the projected point, or None if projection failed.
        """

    @overload
    def project_from_image(self, image_coordinates: list[Point], target_height: float = 0) -> list[Point3d | None]:
        """Project points in image coordinates to a plane in world coordinates.

        :param image_coordinates: The list of image coordinates to project.
        :param target_height: The height of the plane in world coordinates.
        :return: The world coordinates of the projected points, or None if any projection failed.
        """

    @overload
    def project_from_image(self, image_coordinates: FloatArray, target_height: float = 0) -> FloatArray:
        """Project points in image coordinates to a plane in world coordinates.

        :param image_coordinates: (Nx2) The image coordinates to project.
        :param target_height: The height of the plane in world coordinates.
        :return: (Nx3) The world coordinates of the projected points.
        """

    def project_from_image(self, image_coordinates: Point | list[Point] | FloatArray, target_height: float = 0) -> Point3d | None | list[Point3d | None] | FloatArray:
        if isinstance(image_coordinates, Point):
            return self.project_from_image([image_coordinates], target_height=target_height)[0]
        if not isinstance(image_coordinates, np.ndarray):
            image_array = np.array([point.tuple for point in image_coordinates], dtype=np.float64)
            world_array = self.project_from_image(image_array, target_height=target_height)
            return [Point3d(x=point[0], y=point[1], z=point[2]) if not np.isnan(point).any() else None for point in world_array]

        world_extrinsics = self.extrinsics.resolve()
        image_rays = self._points_to_rays(image_coordinates.astype(np.float64).reshape(-1, 1, 2))
        objPoints = image_rays @ world_extrinsics.rotation.matrix.T
        Z = world_extrinsics.z
        t = world_extrinsics.translation_vector
        world_array = t.T - objPoints * (Z - target_height) / objPoints[:, 2:]

        reprojection = self.project_to_image(world_array)
        sign = objPoints[:, -1] * np.sign(Z)
        distance = np.linalg.norm(reprojection - image_coordinates.reshape(-1, 2), axis=1)
        world_array[np.logical_not(np.logical_and(sign < 0, distance < 2)), :] = np.nan

        return world_array

    def _points_to_rays(self, image_points: np.ndarray) -> np.ndarray:
        """Convert image points to rays in homogeneous coordinates with respect to the camera coordinate frame."""
        K = np.array(self.intrinsics.matrix, dtype=np.float32).reshape((3, 3))
        D = np.array(self.intrinsics.distortion)
        if self.intrinsics.model == CameraModel.PINHOLE:
            undistorted = cv2.undistortPoints(image_points, K, D)
        elif self.intrinsics.model == CameraModel.FISHEYE:
            undistorted = cv2.fisheye.undistortPoints(image_points, K, D)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            assert self.intrinsics.omnidir_params is not None, 'Omnidirectional parameters are unset'
            R = self.intrinsics.omnidir_params.rotation.matrix.astype(np.float32)
            xi = np.array(self.intrinsics.omnidir_params.xi, dtype=np.float32)
            undistorted = cv2.omnidir.undistortPoints(image_points, K, D, xi=xi, R=R)
        else:
            raise ValueError(f'Unknown camera model "{self.intrinsics.model}"')

        return cv2.convertPointsToHomogeneous(undistorted).reshape(-1, 3)

    @overload
    def undistort_points(self, image_points: Point, *, crop: bool = False) -> Point:
        """Undistort an image point using the camera calibration.

        :param image_points: The image point to undistort.
        :param crop: Whether cropping is applied to the image during undistortion.
        :return: The undistorted image point.
        """

    @overload
    def undistort_points(self, image_points: list[Point], *, crop: bool = False) -> list[Point]:
        """Undistort image points using the camera calibration.

        :param image_points: The list of image points to undistort.
        :param crop: Whether cropping is applied to the image during undistortion.
        :return: The undistorted image points as a list of Points.
        """

    @overload
    def undistort_points(self, image_points: FloatArray, *, crop: bool = False) -> FloatArray:
        """Undistort image points using the camera calibration.

        :param image_points: (Nx2) The image points to undistort.
        :param crop: Whether cropping is applied to the image during undistortion.
        :return: (Nx2) The undistorted image points.
        """

    def undistort_points(self, image_points: Point | list[Point] | FloatArray, *, crop: bool = False) -> Point | list[Point] | FloatArray:
        if isinstance(image_points, Point):
            return self.undistort_points([image_points], crop=crop)[0]

        if len(image_points) == 0:
            return image_points

        if not isinstance(image_points, np.ndarray):
            image_points = np.array([p.tuple for p in image_points], dtype=np.float32)
            image_points = self.undistort_points(image_points, crop=crop)
            return [Point(x=p[0], y=p[1]) for p in image_points]

        image_points = image_points.reshape(-1, 1, 2)

        K = np.array(self.intrinsics.matrix, dtype=np.float32).reshape((3, 3))
        D = np.array(self.intrinsics.distortion)

        if self.intrinsics.model == CameraModel.PINHOLE:
            if crop:
                log.warning('Cropping is not yet supported for pinhole cameras')
            new_K = self.get_undistorted_camera_matrix(crop=False)
            return cast(FloatArray, cv2.undistortPoints(image_points, K, D, P=new_K, R=np.eye(3)).reshape(-1, 2))
        elif self.intrinsics.model == CameraModel.FISHEYE:
            new_K = self.get_undistorted_camera_matrix(crop=crop)
            return cast(FloatArray, cv2.fisheye.undistortPoints(image_points, K, D, P=new_K).reshape(-1, 2))
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            assert self.intrinsics.omnidir_params is not None, 'Omnidirectional parameters are unset'
            R = np.array(self.intrinsics.omnidir_params.rotation, dtype=np.float32)
            xi = np.array(self.intrinsics.omnidir_params.xi, dtype=np.float32)
            return cast(FloatArray, cv2.omnidir.undistortPoints(image_points, K, D, xi=xi, R=R).reshape(-1, 2))
        else:
            raise ValueError(f'Unknown camera model "{self.intrinsics.model}"')

    FloatArray = NDArray[np.float32] | NDArray[np.float64]

    @overload
    def distort_points(self, image_points: Point, *, crop: bool = False) -> Point:
        """Distort an image point using the camera calibration.

        :param image_points: The image point to distort.
        :param crop: Whether cropping is applied to the image during distortion.
        :return: The distorted image point.
        """

    @overload
    def distort_points(self, image_points: list[Point], *, crop: bool = False) -> list[Point]:
        """Distort a list of image points.
        Note: For pinhole models the redistortion can be off by more than 1px for large distortions.

        :param image_points: The list of image points to distort.
        :param crop: Whether cropping is applied to the image during distortion.
        :return: The distorted image points as a list of Points.
        """

    @overload
    def distort_points(self, image_points: FloatArray, *, crop: bool = False) -> FloatArray:
        """Apply lens distortion to image points using the camera calibration.
        Note: For pinhole models the redistortion can be off by more than 1px for large distortions.

        :param image_points: (Nx2) The image points to distort.
        :param crop: Whether cropping is applied to the image during distortion.
        :return: (Nx2) The distorted image points.
        """

    def distort_points(self, image_points: Point | list[Point] | FloatArray, *, crop: bool = False) -> Point | list[Point] | FloatArray:
        if isinstance(image_points, Point):
            return self.distort_points([image_points], crop=crop)[0]

        if len(image_points) == 0:
            return image_points

        if not isinstance(image_points, np.ndarray):
            image_points = np.array([p.tuple for p in image_points], dtype=np.float32)
            image_points = self.distort_points(image_points, crop=crop)
            return [Point(x=p[0], y=p[1]) for p in image_points]

        if image_points.dtype not in [np.float32, np.float64]:
            raise ValueError('Image point array must be of type float32 or float64')

        image_points = image_points.reshape(-1, 1, 2)

        K = np.array(self.intrinsics.matrix, dtype=np.float32).reshape((3, 3))
        D = np.array(self.intrinsics.distortion)

        if self.intrinsics.model == CameraModel.PINHOLE:
            return self._distort_points_pinhole(image_points).reshape(-1, 2)
        elif self.intrinsics.model == CameraModel.FISHEYE:
            new_K = self.get_undistorted_camera_matrix(crop=crop)
            normalized_points = cv2.undistortPoints(image_points, new_K, None)  # type: ignore
            return cast(FloatArray, cv2.fisheye.distortPoints(normalized_points, K, D).reshape(-1, 2))
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            raise NotImplementedError('Re-distortion for omnidirectional cameras is not supported')
        else:
            raise ValueError(f'Unknown camera model "{self.intrinsics.model}"')

    def _distort_points_pinhole(self, image_points: np.ndarray) -> np.ndarray:
        K = np.array(self.intrinsics.matrix, dtype=np.float32)
        D = np.array(self.intrinsics.distortion, dtype=np.float32)
        new_K = self.get_undistorted_camera_matrix(crop=False)

        # Note: this is a slight hack of available functions.
        # We first normalize the points, then project them
        # In the end this applies only the distortion during projectPoints
        normalized_points = cv2.undistortPoints(image_points, new_K, None)  # type: ignore
        points_3d = cv2.convertPointsToHomogeneous(normalized_points)
        distorted_points, _ = cv2.projectPoints(points_3d, np.zeros(3), np.zeros(3), K, D)
        return distorted_points

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
                                                                           balance=0.5 if crop else 1.0,
                                                                           new_size=(w, h),
                                                                           fov_scale=1)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            if crop:
                log.warning('Cropping is not yet supported for omnidirectional cameras')
            new_K = np.array([[w / 4, 0, w / 2],
                              [0, h / 4, h / 2],
                              [0, 0, 1]])
        else:
            raise ValueError(f'Unknown camera model "{self.intrinsics.model}"')

        return new_K

    @deprecated_function(remove_in_version='0.27.0')
    def undistort_array(self, image_array: np.ndarray, crop: bool = False) -> np.ndarray:
        """
        ### deprecated
            This funciton has been integrated into `undistort_image` and will be removed in Rosys `0.27.0`.

        Undistort an image represented as a numpy array.
        The image is expected to be decoded (in particular not encoded bytes of a JPEG image).

        :param image_array: The image to undistort.
        :param crop: Whether cropping is applied to the image during undistortion.

        :return: The undistorted image.
        """

        return self.undistort_image(image_array, crop=crop)

    @overload
    def undistort_image(self, image: Image, *, crop: bool = False) -> Image:
        """Undistort an image represented as an Image object.

        If you already have the image as an unencoded numpy array, use ``undistort_array`` instead.

        :param image: The image to undistort.
        :param crop: Whether cropping is applied to the image during undistortion.

        :return: The undistorted image.
        """

    @overload
    def undistort_image(self, image: NDArray, *, crop: bool = False) -> NDArray:
        """Undistort an image represented as a numpy array.

        :param image: The image to undistort.
        :param crop: Whether cropping is applied to the image during undistortion.

        :return: The undistorted image.
        """

    def undistort_image(self, image: Image | NDArray, *, crop: bool = False) -> Image | NDArray:

        if isinstance(image, Image):
            return Image(
                camera_id=image.camera_id,
                size=image.size,
                time=image.time,
                data=cv2.imencode('.jpg', self.undistort_image(image.to_array(), crop=crop))[1].tobytes(),
                is_broken=image.is_broken,
                tags=image.tags,
            )

        if image.shape[0] != self.intrinsics.size.height or image.shape[1] != self.intrinsics.size.width:
            log.warning('Image size does not match calibration size (image: %s, calibration: %s)',
                        image.shape, self.intrinsics.size)
            return image

        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        h, w = image.shape[:2]
        if self.intrinsics.model == CameraModel.PINHOLE:
            new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h), centerPrincipalPoint=True)

            dst = cv2.undistort(image, K, D, None, new_K)
            if crop:
                x, y, roi_w, roi_h = roi
                dst = dst[y:y+roi_h, x:x+roi_w]
        elif self.intrinsics.model == CameraModel.FISHEYE:
            new_K = self.get_undistorted_camera_matrix(crop=crop)
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(  # pylint: disable=unpacking-non-sequence
                K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)
            dst = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            flags = cv2.omnidir.RECTIFY_PERSPECTIVE
            new_K = self.get_undistorted_camera_matrix(crop=crop)

            assert self.intrinsics.omnidir_params is not None, 'Omnidirectional parameters are unset'
            R = np.array(self.intrinsics.omnidir_params.rotation.R)
            xi = np.array(self.intrinsics.omnidir_params.xi)
            dst = cv2.omnidir.undistortImage(image, K=K, D=D, xi=xi, Knew=new_K,
                                             flags=flags, new_size=(w, h), R=R)
        else:
            raise ValueError(f'Unknown camera model "{self.intrinsics.model}"')

        return dst

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
