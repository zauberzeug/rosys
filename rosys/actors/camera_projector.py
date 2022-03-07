from typing import Optional
import numpy as np
from ..world import Camera
from .actor import Actor


class CameraProjector(Actor):
    interval: float = 1.0

    async def step(self):
        for camera in self.world.cameras.values():
            if camera.calibration is None or not camera.calibration.is_complete:
                continue
            if camera.projection is not None:
                continue
            self.update_projection(camera)

    @staticmethod
    def update_projection(camera: Camera, rows: int = 12, columns: int = 16):
        c, r = np.meshgrid(np.linspace(0, camera.calibration.intrinsics.size.width, columns),
                           np.linspace(0, camera.calibration.intrinsics.size.height, rows))
        image_points = np.stack((c.flatten(), r.flatten()), axis=1)
        floor_points = camera.calibration.project_array_from_image(image_points).reshape(rows, columns, 3)
        invalid = np.isnan(floor_points).any(axis=2)
        camera.projection = [
            [
                None if invalid[r, c] else [point[0], point[1]]
                for c, point in enumerate(row)
            ]
            for r, row in enumerate(floor_points)
        ]

    @staticmethod
    def allclose(array1: list[list[Optional[list[float]]]],
                 array2: list[list[Optional[list[float]]]]) -> bool:
        is_none1 = [point is None for row in array1 for point in row]
        is_none2 = [point is None for row in array2 for point in row]
        if is_none1 != is_none2:
            return False
        values1 = [point for row in array1 for point in row if point is not None]
        values2 = [point for row in array2 for point in row if point is not None]
        return np.allclose(values1, values2)
