from __future__ import annotations

import enum
import logging
from dataclasses import dataclass, field
from typing import Optional, overload

import cv2
import numpy as np

from ..geometry import Point, Point3d, Rotation
from .image import Image, ImageSize


class CameraModel(enum.Enum):
    PINHOLE = enum.auto()
    FISHEYE = enum.auto()
    OMNIDIRECTIONAL = enum.auto()

    @classmethod
    def from_str(cls, string: str) -> CameraModel:
        if string == 'PINHOLE':
            return CameraModel.PINHOLE
        if string == 'FISHEYE':
            return CameraModel.FISHEYE
        if string == 'OMNIDIRECTIONAL':
            return CameraModel.OMNIDIRECTIONAL
        raise ValueError(f'Unknown camera model "{string}"')


@dataclass(slots=True, kw_only=True)
class Intrinsics:
    model: CameraModel = CameraModel.PINHOLE
    matrix: list[list[float]]
    distortion: list[float]
    xi: float = 0.0
    rotation: Rotation
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

    @classmethod
    def from_dict(cls, data: dict) -> Intrinsics:
        camera_model = CameraModel.from_str(data.get('model', 'PINHOLE'))
        rotation = data.get('rotation', None)
        if rotation is not None:
            n_params = len(rotation)
            if n_params == 3:
                rotation = cv2.Rodrigues(np.array(rotation))[0]
            elif n_params == 9:
                rotation = Rotation(R=rotation)
            else:
                raise ValueError(
                    f'Invalid number of rotation parameters: {n_params}')
        else:
            rotation = Rotation.zero()

        return cls(model=camera_model, matrix=data['matrix'], xi=data.get('xi', 0.0), distortion=data['distortion'], size=ImageSize(**data['size']), rotation=Rotation(R=data.get('rotation', Rotation.zero())))


log = logging.getLogger('rosys.world.calibration')


@dataclass(slots=True, kw_only=True)
class Extrinsics:
    rotation: Rotation = field(
        default_factory=lambda: Rotation.from_euler(np.pi, 0, 0))
    translation: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])

    @classmethod
    def from_dict(cls, data: dict) -> Extrinsics:
        rotation = data.get('rotation', None)
        if rotation is not None:
            n_params = len(rotation)
            if n_params == 3:
                rotation = Rotation(R=cv2.Rodrigues(np.array(rotation))[0].tolist())
            elif n_params == 9:
                rotation = Rotation(R=rotation)
            else:
                raise ValueError(
                    f'Invalid number of rotation parameters: {n_params}')
        else:
            rotation = Rotation.from_euler(np.pi, 0, 0)

        return cls(rotation=rotation, translation=data['translation'])


@dataclass(slots=True, kw_only=True)
class Calibration:
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

    @overload
    def project_to_image(
        self, world_coordinates: Point3d) -> Optional[Point]: ...

    @overload
    def project_to_image(
        self, world_coordinates: np.ndarray) -> np.ndarray: ...

    def project_to_image(self, world_coordinates: Point3d | np.ndarray) -> Optional[Point] | np.ndarray:
        '''
        Projects a point in world coordinates to the image plane.
        This takes into account the camera's intrinsic and extrinsic parameters.
        '''
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

        if self.intrinsics.model == CameraModel.PINHOLE:
            image_array, _ = cv2.projectPoints(world_coordinates, Rod, t, K, D)
        elif self.intrinsics.model == CameraModel.FISHEYE:
            image_array, _ = cv2.fisheye.projectPoints(
                world_coordinates.reshape(-1, 1, 3), Rod, t, K, D)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            xi = self.intrinsics.xi
            image_array, _ = cv2.omnidir.projectPoints(
                world_coordinates.reshape(-1, 1, 3), Rod, t, K, xi, D)
        else:
            raise ValueError(
                f'Unknown camera model "{self.intrinsics.model}"')

        image_array = image_array.reshape(-1, 2)
        world_coordinates = world_coordinates.reshape(-1, 3)

        local_coordinates = (world_coordinates -
                             np.array(self.extrinsics.translation)) @ R
        image_array[local_coordinates[:, 2] < 0, :] = np.nan

        return image_array

    @overload
    def project_from_image(self, image_coordinates: Point,
                           target_height: float = 0) -> Optional[Point3d]: ...

    @overload
    def project_from_image(self, image_coordinates: np.ndarray,
                           target_height: float = 0) -> np.ndarray: ...

    def project_from_image(self, image_coordinates: Point | np.ndarray, target_height: float = 0) -> Optional[Point3d] | np.ndarray:
        '''
        Projects a point in image coordinates to a plane in world xy dimensions at a given height.

        Argments:
            image_coordinates: The image coordinates to project.
            target_height: The height of the plane in world coordinates.

        Returns:
            The world coordinates of the projected point.
        '''

        if isinstance(image_coordinates, Point):
            image_points = np.array(image_coordinates.tuple, dtype=np.float32)
            world_points = self.project_from_image(
                image_points, target_height=target_height)
            if np.isnan(world_points).any():
                return None
            return Point3d(x=world_points[0, 0], y=world_points[0, 1], z=world_points[0, 2])  # pylint: disable=unsubscriptable-object

        image_coordinates__ = self.points_to_rays(
            image_coordinates.astype(np.float32).reshape(-1, 1, 2))
        objPoints = image_coordinates__ @ self.rotation_array.T
        Z = self.extrinsics.translation[-1]
        t = np.array(self.extrinsics.translation)
        world_points = t.T - objPoints * (Z - target_height) / objPoints[:, 2:]
        reprojection = self.project_to_image(world_points)

        sign = objPoints[:, -1] * np.sign(Z)
        distance = np.linalg.norm(
            reprojection - image_coordinates.reshape(-1, 2), axis=1)
        world_points[np.logical_not(np.logical_and(
            sign < 0, distance < 2)), :] = np.nan

        return world_points

    def points_to_rays(self, image_points: np.ndarray) -> np.ndarray:
        '''
        Converts image points to rays in homogeneous coordinates with respect to the camera coordinate frame.
        '''
        K = np.array(self.intrinsics.matrix, dtype=np.float32).reshape((3, 3))
        D = np.array(self.intrinsics.distortion)
        if self.intrinsics.model == CameraModel.PINHOLE:
            undistorted = cv2.undistortPoints(image_points, K, D)
        elif self.intrinsics.model == CameraModel.FISHEYE:
            undistorted = cv2.fisheye.undistortPoints(image_points, K, D)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            R = np.array(self.intrinsics.rotation.R, dtype=np.float32)
            xi = np.array(self.intrinsics.xi, dtype=np.float32)
            undistorted = cv2.omnidir.undistortPoints(
                image_points, K, D, xi=xi, R=R)
        else:
            raise ValueError(
                f'Unknown camera model "{self.intrinsics.model}"')

        return cv2.convertPointsToHomogeneous(undistorted).reshape(-1, 3)

    def undistort_points(self, image_points: np.ndarray, crop: bool = False) -> np.ndarray:
        '''
        Generalized wrapper for undistorting image points.

        Arguments:
            image_points: The image points to undistort.
            crop: Whether cropping is applied to the image during undistortion.

        Returns:
            The undistorted image points.
        '''
        K = np.array(self.intrinsics.matrix, dtype=np.float32).reshape((3, 3))
        D = np.array(self.intrinsics.distortion)
        if self.intrinsics.model == CameraModel.PINHOLE:
            newcameramatrix = self.undistorted_camera_matrix(crop=crop)
            return cv2.undistortPoints(image_points, K, D, P=newcameramatrix, R=np.eye(3))
        elif self.intrinsics.model == CameraModel.FISHEYE:
            newcameramatrix = self.undistorted_camera_matrix(crop=crop)
            return cv2.fisheye.undistortPoints(image_points, K, D, P=newcameramatrix)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            R = np.array(self.intrinsics.rotation.R, dtype=np.float32)
            xi = np.array(self.intrinsics.xi, dtype=np.float32)
            return cv2.omnidir.undistortPoints(image_points, K, D, xi=xi, R=R)

    def distort_points(self, image_points: np.ndarray, crop=False) -> np.ndarray:
        '''
        Generalized wrapper for distorting image points.

        Arguments:
            image_points: The image points to distort.
            crop: Whether cropping is applied to the image during distortion.

        Returns:
            The distorted image points.
        '''

        K = np.array(self.intrinsics.matrix, dtype=np.float32).reshape((3, 3))
        D = np.array(self.intrinsics.distortion)

        if self.intrinsics.model == CameraModel.PINHOLE:
            newcameramatrix = self.undistorted_camera_matrix(crop=crop)
            return cv2.undistortPoints(image_points, K, D, P=newcameramatrix, R=np.eye(3))
        elif self.intrinsics.model == CameraModel.FISHEYE:
            newcameramatrix = self.undistorted_camera_matrix(crop=crop)
            normalized_points = np.linalg.inv(newcameramatrix) @ \
                cv2.convertPointsToHomogeneous(image_points).reshape(-1, 3).T
            normalized_points = cv2.convertPointsFromHomogeneous(
                normalized_points.T).reshape(-1, 1, 2)
            return cv2.fisheye.distortPoints(normalized_points, K, D)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            raise NotImplementedError(
                'Re-distortion for omnidirectional cameras is not supported')

    @staticmethod
    def from_points(
        world_points: list[Point3d], image_points: list[Point], image_size: ImageSize, f0: float,
            rational_model: bool = False, camera_model: CameraModel = CameraModel.PINHOLE) -> Calibration:
        '''
        Estimates the camera calibration from corresponding world and image points.

        Arguments:
            world_points: The observed points in 3D world coordinates.
            image_points: The observed points in 2D image coordinates.
            image_size: The size of the image.
            f0: An initial guess for the focal length.
            rational_model: Whether to use the rational camera model (only applies to pinhole cameras).
            fisheye: Whether to use the fisheye camera model.

        Returns:
            The estimated camera calibration.
        '''

        if rational_model and camera_model != CameraModel.PINHOLE:
            raise ValueError(
                'Rational model is only supported for pinhole cameras')

        world_point_array = [np.array(
            [p.tuple for p in world_points], dtype=np.float32).reshape((-1, 1, 3))]
        image_point_array = [np.array(
            [p.tuple for p in image_points], dtype=np.float32).reshape((-1, 1, 2))]
        K0 = np.array([[f0, 0, image_size.width / 2], [0, f0,
                      image_size.height / 2], [0, 0, 1]], dtype=np.float32)
        D0 = np.array([0.1, 0.4, -0.5, 0.2], dtype=np.float32).reshape(1, 4)

        xi: float = 0.0

        if camera_model == CameraModel.PINHOLE:
            flags = cv2.CALIB_USE_INTRINSIC_GUESS
            if rational_model:
                flags |= cv2.CALIB_RATIONAL_MODEL
            _, K, D, rvecs, tvecs = cv2.calibrateCamera(
                objectPoints=world_point_array, imagePoints=image_point_array, imageSize=image_size.tuple, cameraMatrix=K0, distCoeffs=None,
                flags=flags, criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
        elif camera_model == CameraModel.FISHEYE:
            flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | \
                cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
            _, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                objectPoints=world_point_array, imagePoints=image_point_array, image_size=image_size.tuple, K=K0, D=D0, flags=flags,
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
            if D.size != 4:
                raise ValueError(
                    f'Fisheye calibration failed (got invalid distortion coefficients D="{D}")')
        elif camera_model == CameraModel.OMNIDIRECTIONAL:
            flags = cv2.omnidir.CALIB_USE_GUESS + cv2.omnidir.CALIB_FIX_SKEW
            world_point_array[0] = world_point_array[0].astype(np.float64)
            image_point_array[0] = image_point_array[0].astype(np.float64)
            rms, K, xi_arr, D, rvecs, tvecs, status = cv2.omnidir.calibrate(
                objectPoints=world_point_array, imagePoints=image_point_array, size=image_size.tuple, K=K0, xi=np.array([0.0], dtype=np.float32), D=D0, flags=flags,
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
            xi = xi_arr[0]
            if D.size != 4:
                raise ValueError(
                    f'Omnidirectional calibration failed (got invalid distortion coefficients D="{D}")')
        else:
            raise ValueError(f'Unknown camera model "{camera_model}"')

        rotation0 = Rotation(R=np.eye(3).tolist())
        intrinsics = Intrinsics(matrix=K.tolist(), distortion=D.tolist()[
            0], rotation=rotation0, size=image_size, model=camera_model, xi=xi)

        rotation = Rotation.from_rvec(rvecs[0]).T
        translation = (-np.array(rotation.R).dot(tvecs)).flatten().tolist()
        extrinsics = Extrinsics(rotation=rotation, translation=translation)

        return Calibration(intrinsics=intrinsics, extrinsics=extrinsics)

    def undistorted_camera_matrix(self, crop=False) -> np.ndarray:
        '''
        Computes the camera matrix for the undistorted image.

        Arguments:
            crop: Whether cropping is applied to the image during undistortion.

        Returns:
            The camera matrix for the undistorted image.
        '''
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        h, w = self.intrinsics.size.height, self.intrinsics.size.width
        balance = 0.0 if crop else 1.0
        if self.intrinsics.model == CameraModel.PINHOLE:
            newcameramtx, _ = cv2.getOptimalNewCameraMatrix(
                K, D, (w, h), 1, (w, h), centerPrincipalPoint=True)
        elif self.intrinsics.model == CameraModel.FISHEYE:
            newcameramtx = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K, D, (w, h), np.eye(3), balance=balance, new_size=(w, h), fov_scale=1)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            if crop:
                logging.warning(
                    'Cropping is not yet supported for omnidirectional cameras')
            new_size = ImageSize(width=w, height=h)
            newcameramtx = np.array([[new_size.width/4, 0, new_size.width/2],
                                     [0, new_size.height/4, new_size.height/2],
                                     [0, 0, 1]])
        else:
            raise ValueError(
                f'Unknown camera model "{self.intrinsics.model}"')

        return newcameramtx

    def undistort_array(self, image_array: np.ndarray, crop=False) -> np.ndarray:
        '''
        Undistorts an image represented as a numpy array.
        The image is expected to be decoded (i.e. not encoded bytes of a jpg image).

        Arguments:
            image_array: The image to undistort.
            crop: Whether cropping is applied to the image during undistortion.

        Returns:
            The undistorted image.
        '''
        if image_array.shape[0] != self.intrinsics.size.height or image_array.shape[1] != self.intrinsics.size.width:
            log.warning('Image size does not match calibration size (image: %s, calibration: %s)',
                        image_array.shape, self.intrinsics.size)
            return image_array

        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        h, w = image_array.shape[:2]
        if self.intrinsics.model == CameraModel.PINHOLE:
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
                K, D, (w, h), 1, (w, h), centerPrincipalPoint=True)

            dst = cv2.undistort(image_array, K, D, None, newcameramtx)
            if crop and self.intrinsics.model == CameraModel.PINHOLE:
                x, y, w, h = roi
                dst = dst[y:y+h, x:x+w]
        elif self.intrinsics.model == CameraModel.FISHEYE:
            # 'balance' controls the trade-off between FOV and cropping (0=full crop, no black edges; 1=full FOV, might have black edges)
            balance = 0.0 if crop else 1.0
            newcameramtx = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K, D, (w, h), np.eye(3), balance=balance, new_size=(w, h), fov_scale=1)

            new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K, D, (w, h), np.eye(3), balance=balance)

            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)
            dst = cv2.remap(image_array, map1, map2,
                            interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        elif self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            new_size = ImageSize(width=w, height=h)
            flags = cv2.omnidir.RECTIFY_PERSPECTIVE
            new_K = self.undistorted_camera_matrix(crop=crop)

            R = np.array(self.intrinsics.rotation.R)
            xi = np.array(self.intrinsics.xi)
            dst = cv2.omnidir.undistortImage(image_array, K=K, D=D, xi=xi, Knew=new_K,
                                             flags=flags, new_size=(w, h), R=R)
        else:
            raise ValueError(
                f'Unknown camera model "{self.intrinsics.model}"')

        return dst

    def undistorted_size(self, size: ImageSize) -> ImageSize:
        '''
        Computes the size of the undistorted image after cropping.

        Arguments:
            size: The size of the distorted image.

        Returns:
            The size of the undistorted image.
        '''
        if self.intrinsics.model == CameraModel.FISHEYE:
            return size
        if self.intrinsics.model == CameraModel.OMNIDIRECTIONAL:
            return size
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        h, w = size.height, size.width
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            K, D, (w, h), 1, (w, h), centerPrincipalPoint=True)
        x, y, w, h = roi
        return ImageSize(width=w, height=h)

    def undistort_image(self, image: Image) -> Image:
        '''
        Undistorts an image represented as an Image object.
        If you already have the image as an unencoded numpy array, use undistort_array instead.

        Arguments:
            image: The image to undistort.

        Returns:
            The undistorted image.
        '''

        array = image.to_array()
        if array is None:
            return image

        array = self.undistort_array(array)
        data = cv2.imencode('.jpg', array)[1].tobytes()

        return Image(camera_id=image.camera_id, size=image.size, time=image.time, data=data, is_broken=image.is_broken, tags=image.tags)
