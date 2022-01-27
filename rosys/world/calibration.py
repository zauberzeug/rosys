from pydantic import BaseModel
from typing import Optional
import numpy as np
import cv2
from .point import Point
from .point3d import Point3d
from .rotation import Rotation
from .image import ImageSize


class Intrinsics(BaseModel):
    matrix: list[list[float]]
    distortion: list[float]
    rotation: Rotation
    size: ImageSize


class Extrinsics(BaseModel):
    tilt: Optional[Rotation]
    yaw: float = 0
    translation: list[float] = [0, 0, 1]


class Calibration(BaseModel):
    intrinsics: Intrinsics
    extrinsics: Extrinsics = Extrinsics()

    @property
    def rotation(self) -> Rotation:
        tilt = self.extrinsics.tilt or Rotation.zero()
        return Rotation.from_euler(0, 0, self.extrinsics.yaw) * tilt * self.intrinsics.rotation

    @property
    def is_complete(self) -> bool:
        return self.extrinsics.tilt is not None

    def project_to_image(self, world_point: Point3d) -> Point:
        world_points = np.array([world_point.tuple])
        R = np.array(self.rotation.R)
        Rod = cv2.Rodrigues(R.T)[0]
        t = -R.T @ self.extrinsics.translation
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        image_points, _ = cv2.projectPoints(world_points, Rod, t, K, D)
        return Point(x=image_points[0, 0, 0], y=image_points[0, 0, 1])

    def project_from_image(self, image_point: Point, target_height: float = 0) -> Optional[Point3d]:
        K = np.array(self.intrinsics.matrix)
        D = np.array(self.intrinsics.distortion)
        image_points = np.array(image_point.tuple, dtype=np.float32).reshape(1, 1, 2)
        image_points_ = cv2.undistortPoints(image_points, K, D).reshape(-1, 2)
        image_points__ = cv2.convertPointsToHomogeneous(image_points_).reshape(-1, 3)
        objPoints = image_points__ @ self.rotation.T.R
        Z = self.extrinsics.translation[-1]
        t = np.array(self.extrinsics.translation)
        floorPoints = t.T - objPoints * Z / objPoints[:, 2:]

        reprojection = self.project_to_image(Point3d(x=floorPoints[0, 0], y=floorPoints[0, 1], z=0))
        if objPoints[0, -1] * np.sign(Z) > 0 or reprojection.distance(image_point) > 2:
            return None

        X, Y, _ = floorPoints[0]
        X_ = (X - t[0]) / (t[2] + target_height) * t[2] + t[0]
        Y_ = (Y - t[1]) / (t[2] + target_height) * t[2] + t[1]
        return Point3d(x=X_, y=Y_, z=target_height)

    @staticmethod
    def from_points(world_points: list[Point3d], image_points: list[Point], image_size: ImageSize = ImageSize(width=1600, height=1200), f0=500):
        world_points = [(p.x, p.y, p.z) for p in world_points]
        world_points = np.array(world_points, dtype=np.float32).reshape(1, -1, 3)
        image_points = [(p.x, p.y) for p in image_points]
        image_points = np.array(image_points, dtype=np.float32).reshape(1, -1, 2)
        K0 = np.array([[f0, 0, image_size.width / 2], [0, f0, image_size.height / 2], [0, 0, 1]], dtype=np.float32)
        flags = cv2.CALIB_USE_INTRINSIC_GUESS
        ret, K, D, rvecs, tvecs = cv2.calibrateCamera(world_points,
                                                      image_points,
                                                      (image_size.width, image_size.height),
                                                      K0, None, flags=flags)

        rot0 = Rotation.from_rvec(rvecs[0]).T
        intrinsics = Intrinsics(matrix=K.tolist(), distortion=D.tolist()[0], rotation=rot0,
                                size=ImageSize(width=image_size.width, height=image_size.height))

        translation = (-np.array(rot0.R).dot(tvecs)).tolist()
        translation = [v[0][0] for v in translation]
        extrinsics = Extrinsics(tilt=Rotation(R=np.eye(3).tolist()), translation=translation)
        return Calibration(intrinsics=intrinsics, extrinsics=extrinsics)
