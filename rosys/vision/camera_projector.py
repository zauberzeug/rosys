from copy import deepcopy
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rosys

from .calibration import Calibration
from .camera import Camera
from .camera_provider import CameraProvider

ProjectionCoordinates = list[list[Optional[list[float]]]]


@dataclass(slots=True, kw_only=True)
class Projection:
    camera_id: str
    camera_calibration: Calibration
    coordinates: ProjectionCoordinates


class CameraProjector:
    '''The camera projector computes a grid of projected image points on the ground plane.

    It is mainly used for visualization purposes.
    '''

    def __init__(self, camera_provider: CameraProvider) -> None:
        self.camera_provider = camera_provider

        self.projections: dict[str, Projection] = {}

        rosys.on_repeat(self.step, 1.0)

    async def step(self) -> None:
        for id in list(self.projections):
            if id not in self.camera_provider.cameras:
                del self.projections[id]

        for id, camera in self.camera_provider.cameras.items():
            if not camera.calibration:
                continue
            if id in self.projections and self.projections[id].camera_calibration == camera.calibration:
                continue
            self.projections[id] = Projection(
                camera_id=id,
                camera_calibration=deepcopy(camera.calibration),
                coordinates=self.project(camera),
            )

    @staticmethod
    def project(camera: Camera, rows: int = 12, columns: int = 16) -> ProjectionCoordinates:
        c, r = np.meshgrid(np.linspace(0, camera.calibration.intrinsics.size.width, columns),
                           np.linspace(0, camera.calibration.intrinsics.size.height, rows))
        image_points = np.stack((c.flatten(), r.flatten()), axis=1)
        floor_points = camera.calibration.project_array_from_image(image_points).reshape(rows, columns, 3)
        invalid = np.isnan(floor_points).any(axis=2)
        return [
            [
                None if invalid[r, c] else [point[0], point[1]]
                for c, point in enumerate(row)
            ]
            for r, row in enumerate(floor_points)
        ]

    @staticmethod
    def allclose(array1: ProjectionCoordinates, array2: ProjectionCoordinates) -> bool:
        is_none1 = [point is None for row in array1 for point in row]
        is_none2 = [point is None for row in array2 for point in row]
        if is_none1 != is_none2:
            return False
        values1 = [point for row in array1 for point in row if point is not None]
        values2 = [point for row in array2 for point in row if point is not None]
        return np.allclose(values1, values2)
