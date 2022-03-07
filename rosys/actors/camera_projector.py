from typing import Optional
import numpy as np
from rosys.world import Point
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
        camera.projection = [[None for _ in range(columns)] for _ in range(rows)]
        for row, j in enumerate(np.linspace(0, camera.calibration.intrinsics.size.height, rows)):
            for column, i in enumerate(np.linspace(0, camera.calibration.intrinsics.size.width, columns)):
                point = camera.calibration.project_from_image(Point(x=i, y=j))
                if point is not None:
                    camera.projection[row][column] = point.tuple[:2]

    @staticmethod
    def allclose(array1: list[list[list[Optional[float]]]],
                 array2: list[list[list[Optional[float]]]]) -> bool:
        is_none1 = [point is None for row in array1 for point in row]
        is_none2 = [point is None for row in array2 for point in row]
        if is_none1 != is_none2:
            return False
        values1 = [point for row in array1 for point in row if point is not None]
        values2 = [point for row in array2 for point in row if point is not None]
        return np.allclose(values1, values2)
