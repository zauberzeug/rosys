from pydantic import BaseModel
from typing import Any, Optional
import numpy as np
import cv2
from .image import ImageSize
from .point import Point
from .point3d import Point3d
from .rotation import Rotation


class Intrinsics(BaseModel):
    matrix: list[list[float]]
    distortion: list[float]
    rotation: Rotation
    size: ImageSize


class Extrinsics(BaseModel):
    tilt: Optional[Rotation]
    yaw: float = 0.0
    translation: list[float] = [0.0, 0.0, 1.0]


class Calibration(BaseModel):
    intrinsics: Intrinsics
    extrinsics: Extrinsics = Extrinsics()

    @property
    def rotation(self) -> Rotation:
        tilt = self.extrinsics.tilt or Rotation.zero()
        return Rotation.from_euler(0, 0, self.extrinsics.yaw) * tilt * self.intrinsics.rotation

    @property
    def rotation_array(self) -> Any:
        tilt = self.extrinsics.tilt or Rotation.zero()
        yaw = self.extrinsics.yaw
        return np.dot([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]], tilt.R) @ self.intrinsics.rotation.R

    @property
    def is_complete(self) -> bool:
        return self.extrinsics.tilt is not None

    def project_to_image(self, world_point: Point3d) -> Point:
        world_points = np.array([world_point.tuple])
        R = self.rotation_array
        Rod = cv2.Rodrigues(R.T)[0]
        t = -R.T @ self.extrinsics.translation
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        image_points, _ = cv2.projectPoints(world_points, Rod, t, K, D)
        return Point(x=image_points[0, 0, 0], y=image_points[0, 0, 1])

    def project_array_to_image(self, world_points: Any) -> Any:
        R = self.rotation_array
        Rod = cv2.Rodrigues(R.T)[0]
        t = -R.T @ self.extrinsics.translation
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        image_points, _ = cv2.projectPoints(world_points, Rod, t, K, D)
        return image_points.reshape(-1, 2)

    def project_from_image(self, image_point: Point, target_height: float = 0) -> Optional[Point3d]:
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        image_points = np.array(image_point.tuple, dtype=np.float32).reshape(1, 1, 2)
        image_points_ = cv2.undistortPoints(image_points, K, D).reshape(-1, 2)
        image_points__ = cv2.convertPointsToHomogeneous(image_points_).reshape(-1, 3)
        objPoints = image_points__ @ self.rotation_array.T
        Z = self.extrinsics.translation[-1]
        t = np.array(self.extrinsics.translation)
        floor_points = t.T - objPoints * (Z - target_height) / objPoints[:, 2:]

        reprojection = self.project_to_image(Point3d(x=floor_points[0, 0], y=floor_points[0, 1], z=target_height))
        if objPoints[0, -1] * np.sign(Z) > 0 or reprojection.distance(image_point) > 2:
            return None

        return Point3d(x=floor_points[0, 0], y=floor_points[0, 1], z=target_height)

    def project_array_from_image(self, image_points: Any, target_height: float = 0) -> Any:
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
    def from_points(world_points: list[Point3d], image_points: list[Point], image_size: ImageSize, f0: float):
        world_points = np.array([p.tuple for p in world_points], dtype=np.float32).reshape(1, -1, 3)
        image_points = np.array([p.tuple for p in image_points], dtype=np.float32).reshape(1, -1, 2)
        K0 = np.array([[f0, 0, image_size.width / 2], [0, f0, image_size.height / 2], [0, 0, 1]], dtype=np.float32)
        flags = cv2.CALIB_USE_INTRINSIC_GUESS
        _, K, D, rvecs, tvecs = cv2.calibrateCamera(world_points, image_points, image_size.tuple, K0, None, flags=flags)

        rotation0 = Rotation(R=np.eye(3).tolist())
        intrinsics = Intrinsics(matrix=K.tolist(), distortion=D.tolist()[0], rotation=rotation0, size=image_size)

        rotation = Rotation.from_rvec(rvecs[0]).T
        translation = (-np.array(rotation.R).dot(tvecs)).flatten().tolist()
        extrinsics = Extrinsics(tilt=rotation, translation=translation)

        return Calibration(intrinsics=intrinsics, extrinsics=extrinsics)
