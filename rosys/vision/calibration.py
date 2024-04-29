from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Optional, overload

import cv2
import numpy as np

from ..geometry import Point, Point3d, Rotation
from .image import Image, ImageSize


@dataclass(slots=True, kw_only=True)
class Intrinsics:
    fisheye: bool = False
    matrix: list[list[float]]
    distortion: list[float]
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
        return cls(fisheye=bool(data.get('fisheye', False)), matrix=data['matrix'], distortion=data['distortion'], size=ImageSize(**data['size']), rotation=Rotation(R=data.get('rotation', Rotation.zero())))


log = logging.getLogger('rosys.world.calibration')


@dataclass(slots=True, kw_only=True)
class Extrinsics:
    rotation: Rotation = field(default_factory=lambda: Rotation.from_euler(np.pi, 0, 0))
    translation: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])

    @classmethod
    def from_dict(cls, data: dict) -> Extrinsics:
        return cls(rotation=Rotation(R=data['rotation']), translation=data['translation'])


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

    @overload
    def project_to_image(self, world_coordinates: Point3d) -> Optional[Point]: ...

    @overload
    def project_to_image(self, world_coordinates: np.ndarray) -> np.ndarray: ...

    def project_to_image(self, world_coordinates: Point3d | np.ndarray) -> Optional[Point] | np.ndarray:
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
        if self.intrinsics.fisheye:
            image_array, _ = cv2.fisheye.projectPoints(
                world_coordinates.reshape(-1, 1, 3), Rod, t, K, D)
        else:
            image_array, _ = cv2.projectPoints(world_coordinates, Rod, t, K, D)

        image_array = image_array.reshape(-1, 2)
        world_coordinates = world_coordinates.reshape(-1, 3)

        local_coordinates = (world_coordinates - np.array(self.extrinsics.translation)) @ R
        image_array[local_coordinates[:, 2] < 0, :] = np.nan

        return image_array

    @overload
    def project_from_image(self, image_coordinates: Point, target_height: float = 0) -> Optional[Point3d]: ...

    @overload
    def project_from_image(self, image_coordinates: np.ndarray, target_height: float = 0) -> np.ndarray: ...

    def project_from_image(self, image_coordinates: Point | np.ndarray, target_height: float = 0) -> Optional[Point3d] | np.ndarray:
        if isinstance(image_coordinates, Point):
            image_points = np.array(image_coordinates.tuple, dtype=np.float32)
            world_points = self.project_from_image(image_points, target_height=target_height)
            if np.isnan(world_points).any():
                return None
            return Point3d(x=world_points[0, 0], y=world_points[0, 1], z=world_points[0, 2])  # pylint: disable=unsubscriptable-object

        image_coordinates__ = self.points_to_rays(image_coordinates.astype(np.float32).reshape(-1, 1, 2))
        objPoints = image_coordinates__ @ self.rotation_array.T
        Z = self.extrinsics.translation[-1]
        t = np.array(self.extrinsics.translation)
        world_points = t.T - objPoints * (Z - target_height) / objPoints[:, 2:]

        # reprojection = self.project_to_image(world_points)
        # sign = objPoints[:, -1] * np.sign(Z)
        # distance = np.linalg.norm(reprojection - image_coordinates, axis=1)
        # world_points[np.logical_not(np.logical_and(sign < 0, distance < 2)), :] = np.nan

        return world_points

    def points_to_rays(self, image_points: np.ndarray) -> np.ndarray:
        K = np.array(self.intrinsics.matrix, dtype=np.float32).reshape((3, 3))
        D = np.array(self.intrinsics.distortion)
        if self.intrinsics.fisheye:
            undistorted = cv2.fisheye.undistortPoints(image_points, K, D)
        else:
            undistorted = cv2.undistortPoints(image_points, K, D)

        return cv2.convertPointsToHomogeneous(undistorted).reshape(-1, 3)

    def undistort_points(self, image_points: np.ndarray, crop=False) -> np.ndarray:
        K = np.array(self.intrinsics.matrix, dtype=np.float32).reshape((3, 3))
        D = np.array(self.intrinsics.distortion)
        if self.intrinsics.fisheye:
            newcameramatrix = self.undistorted_camera_matrix(crop=crop)
            return cv2.fisheye.undistortPoints(image_points, K, D, P=newcameramatrix)
        else:
            newcameramatrix = self.undistorted_camera_matrix(crop=crop)
            return cv2.undistortPoints(image_points, K, D, P=newcameramatrix, R=np.eye(3))

    def distort_points(self, image_points: np.ndarray, crop=False) -> np.ndarray:
        K = np.array(self.intrinsics.matrix, dtype=np.float32).reshape((3, 3))
        D = np.array(self.intrinsics.distortion)
        if self.intrinsics.fisheye:
            newcameramatrix = self.undistorted_camera_matrix(crop=crop)
            normalized_points = np.linalg.inv(newcameramatrix) @ \
                cv2.convertPointsToHomogeneous(image_points).reshape(-1, 3).T
            normalized_points = cv2.convertPointsFromHomogeneous(
                normalized_points.T).reshape(-1, 1, 2)
            return cv2.fisheye.distortPoints(normalized_points, K, D)
        else:
            newcameramatrix = self.undistorted_camera_matrix(crop=crop)
            return cv2.undistortPoints(image_points, K, D, P=newcameramatrix, R=np.eye(3))

    @staticmethod
    def from_points(
        world_points: list[Point3d], image_points: list[Point], image_size: ImageSize, f0: float,
            rational_model: bool = False, fisheye: bool = False) -> Calibration:
        if fisheye and rational_model:
            raise ValueError('Rational model is not supported for fisheye cameras')

        world_point_array = [np.array(
            [p.tuple for p in world_points], dtype=np.float32).reshape((-1, 1, 3))]
        image_point_array = [np.array(
            [p.tuple for p in image_points], dtype=np.float32).reshape((-1, 1, 2))]
        K0 = np.array([[f0, 0, image_size.width / 2], [0, f0,
                      image_size.height / 2], [0, 0, 1]], dtype=np.float32)
        D0 = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32).reshape(1, 4)

        if fisheye:
            flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | \
                cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
            _, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                objectPoints=world_point_array, imagePoints=image_point_array, image_size=image_size.tuple, K=K0, D=D0, flags=flags,
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
        else:
            flags = cv2.CALIB_USE_INTRINSIC_GUESS
            if rational_model:
                flags |= cv2.CALIB_RATIONAL_MODEL
            _, K, D, rvecs, tvecs = cv2.calibrateCamera(
                objectPoints=world_point_array, imagePoints=image_point_array, imageSize=image_size.tuple, cameraMatrix=K0, distCoeffs=None,
                flags=flags)

        rotation0 = Rotation(R=np.eye(3).tolist())
        intrinsics = Intrinsics(matrix=K.tolist(), distortion=D.tolist()[
            0], rotation=rotation0, size=image_size, fisheye=fisheye)

        rotation = Rotation.from_rvec(rvecs[0]).T
        translation = (-np.array(rotation.R).dot(tvecs)).flatten().tolist()
        extrinsics = Extrinsics(rotation=rotation, translation=translation)

        return Calibration(intrinsics=intrinsics, extrinsics=extrinsics)

    def undistorted_camera_matrix(self, crop=False) -> np.ndarray:
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        h, w = self.intrinsics.size.height, self.intrinsics.size.width
        balance = 0.0 if crop else 1.0
        if self.intrinsics.fisheye:
            newcameramtx = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K, D, (w, h), np.eye(3), balance=balance, new_size=(w, h), fov_scale=1)
        else:
            newcameramtx, _ = cv2.getOptimalNewCameraMatrix(
                K, D, (w, h), 1, (w, h), centerPrincipalPoint=True)
        return newcameramtx

    def undistort_array(self, image_array: np.ndarray, crop=False) -> np.ndarray:
        if image_array.shape[0] != self.intrinsics.size.height or image_array.shape[1] != self.intrinsics.size.width:
            log.warning('Image size does not match calibration size (image: %s, calibration: %s)',
                        image_array.shape, self.intrinsics.size)
            return image_array

        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        h, w = image_array.shape[:2]
        if self.intrinsics.fisheye:
            # 'balance' controls the trade-off between FOV and cropping (0=full crop, no black edges; 1=full FOV, might have black edges)
            balance = 0.0 if crop else 1.0
            newcameramtx = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K, D, (w, h), np.eye(3), balance=balance, new_size=(w, h), fov_scale=1)

            new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K, D, (w, h), np.eye(3), balance=balance)

            map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)
            dst = cv2.remap(image_array, map1, map2,
                            interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        else:
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
                K, D, (w, h), 1, (w, h), centerPrincipalPoint=True)

            dst = cv2.undistort(image_array, K, D, None, newcameramtx)
            if crop and not self.intrinsics.fisheye:
                x, y, w, h = roi
                dst = dst[y:y+h, x:x+w]
        return dst

    def undistorted_size(self, size: ImageSize) -> ImageSize:
        if self.intrinsics.fisheye:
            return size

        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        h, w = size.height, size.width
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            K, D, (w, h), 1, (w, h), centerPrincipalPoint=True)
        x, y, w, h = roi
        return ImageSize(width=w, height=h)

    def undistort_image(self, image: Image) -> Image:
        array = image.to_array()
        if array is None:
            return image

        array = self.undistort_array(array)
        data = cv2.imencode('.jpg', array)[1].tobytes()

        return Image(camera_id=image.camera_id, size=image.size, time=image.time, data=data, is_broken=image.is_broken, tags=image.tags)
