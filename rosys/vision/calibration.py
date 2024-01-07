from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from ..geometry import Point, Point3d, Rotation
from .image import ImageSize


@dataclass(slots=True, kw_only=True)
class Intrinsics:
    matrix: list[list[float]]
    distortion: list[float]
    rotation: Rotation
    size: ImageSize

    @staticmethod
    def create_default(width: int = 800, height: int = 600, *, focal_length: float = 570) -> Intrinsics:
        size = ImageSize(width=width, height=height)
        K: list[list[float]] = [[focal_length, 0, size.width / 2], [0, focal_length, size.height / 2], [0, 0, 1]]
        D: list[float] = [0] * 5
        rotation = Rotation.zero()
        return Intrinsics(matrix=K, distortion=D, rotation=rotation, size=size)


log = logging.getLogger('rosys.world.calibration')


@dataclass(slots=True, kw_only=True)
class Extrinsics:
    rotation: Rotation = field(default_factory=lambda: Rotation.from_euler(np.pi, 0, 0))
    translation: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])


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

    def project_to_image(self, world_point: Point3d) -> Point:
        world_points = np.array([world_point.tuple], dtype=np.float32)
        R = self.rotation_array
        Rod = cv2.Rodrigues(R.T)[0]
        t = -R.T @ self.extrinsics.translation
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion, dtype=float)
        image_points, _ = cv2.projectPoints(world_points, Rod, t, K, D)
        return Point(x=image_points[0, 0, 0], y=image_points[0, 0, 1])

    def project_array_to_image(self, world_points: np.ndarray) -> np.ndarray:
        R = self.rotation_array
        Rod = cv2.Rodrigues(R.T)[0]
        t = -R.T @ self.extrinsics.translation
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion, dtype=float)
        image_points, _ = cv2.projectPoints(world_points, Rod, t, K, D)
        return image_points.reshape(-1, 2)

    def project_from_image(self, image_point: Point, target_height: float = 0) -> Optional[Point3d]:
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        image_points = np.array(image_point.tuple, dtype=np.float32).reshape((1, 1, 2))
        image_points_ = cv2.undistortPoints(image_points, K, D).reshape(-1, 2)
        image_points__ = cv2.convertPointsToHomogeneous(image_points_).reshape(-1, 3)
        objPoints = image_points__ @ self.rotation_array.T
        Z = self.extrinsics.translation[-1]
        t = np.array(self.extrinsics.translation)
        floor_points = t.T - objPoints * (Z - target_height) / objPoints[:, 2:]

        reprojection = self.project_to_image(Point3d(x=floor_points[0, 0], y=floor_points[0, 1], z=target_height))
        if objPoints[0, -1] * np.sign(Z) > 0 or reprojection.distance(image_point) > 2:
            log.warning(f'reprojection failed with {reprojection.distance(image_point)} px')
            return None

        return Point3d(x=floor_points[0, 0], y=floor_points[0, 1], z=target_height)

    def project_array_from_image(self, image_points: np.ndarray, target_height: float = 0) -> np.ndarray:
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        image_points_ = cv2.undistortPoints(image_points.astype(np.float32), K, D).reshape(-1, 2)
        image_points__ = cv2.convertPointsToHomogeneous(image_points_).reshape(-1, 3)
        objPoints = image_points__ @ self.rotation_array.T
        Z = self.extrinsics.translation[-1]
        t = np.array(self.extrinsics.translation)
        floor_points = t.T - objPoints * (Z - target_height) / objPoints[:, 2:]

        reprojection = self.project_array_to_image(floor_points)
        sign = objPoints[:, -1] * np.sign(Z)
        distance = np.linalg.norm(reprojection - image_points, axis=1)
        floor_points[np.logical_or(sign > 0, distance > 2), :] = np.nan

        return floor_points

    @staticmethod
    def from_points(
            world_points: list[Point3d], image_points: list[Point], image_size: ImageSize, f0: float) -> Calibration:
        world_point_array = np.array([p.tuple for p in world_points], dtype=np.float32).reshape((1, -1, 3))
        image_point_array = np.array([p.tuple for p in image_points], dtype=np.float32).reshape((1, -1, 2))
        K0 = np.array([[f0, 0, image_size.width / 2], [0, f0, image_size.height / 2], [0, 0, 1]], dtype=np.float32)
        _, K, D, rvecs, tvecs = cv2.calibrateCamera(world_point_array, image_point_array, image_size.tuple, K0, None,
                                                    flags=cv2.CALIB_USE_INTRINSIC_GUESS)

        rotation0 = Rotation(R=np.eye(3).tolist())
        intrinsics = Intrinsics(matrix=K.tolist(), distortion=D.tolist()[0], rotation=rotation0, size=image_size)

        rotation = Rotation.from_rvec(rvecs[0]).T
        translation = (-np.array(rotation.R).dot(tvecs)).flatten().tolist()
        extrinsics = Extrinsics(rotation=rotation, translation=translation)

        return Calibration(intrinsics=intrinsics, extrinsics=extrinsics)
