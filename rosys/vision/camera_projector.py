from copy import deepcopy
from dataclasses import dataclass

import numpy as np

from .. import rosys
from .calibratable_camera_provider import CalibratableCameraProvider
from .calibration import Calibration
from .camera import CalibratableCamera

ProjectionCoordinates = list[list[list[float] | None]]


@dataclass(slots=True, kw_only=True)
class Projection:
    camera_id: str
    camera_calibration: Calibration
    coordinates: ProjectionCoordinates


class CameraProjector:
    """The camera projector computes a grid of projected image points on the ground plane.

    It is mainly used for visualization purposes.
    """

    def __init__(self, camera_provider: CalibratableCameraProvider, *, interval: float = 1.0) -> None:
        self.camera_provider = camera_provider

        self.projections: dict[str, Projection] = {}

        rosys.on_repeat(self.step, interval)

    async def step(self) -> None:
        for id_ in list(self.projections):
            if id_ not in self.camera_provider.cameras:
                del self.projections[id_]

        for id_, camera in self.camera_provider.cameras.items():
            if not camera.calibration:
                continue
            self.projections[id_] = Projection(
                camera_id=id_,
                camera_calibration=deepcopy(camera.calibration),
                coordinates=self.project(camera),
            )

    @staticmethod
    def project(camera: CalibratableCamera, rows: int = 12, columns: int = 16) -> ProjectionCoordinates:
        assert camera.calibration is not None
        c, r = np.meshgrid(np.linspace(0, camera.calibration.intrinsics.size.width, columns),
                           np.linspace(0, camera.calibration.intrinsics.size.height, rows))
        image_points = np.stack((c.flatten(), r.flatten()), axis=1)
        floor_points = camera.calibration.project_from_image(image_points).reshape(rows, columns, 3)
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
