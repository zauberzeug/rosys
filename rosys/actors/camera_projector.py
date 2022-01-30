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

    def update_projection(self, camera: Camera, rows: int = 12, columns: int = 16):
        camera.projection = [[None for _ in range(columns)] for _ in range(rows)]
        for row, j in enumerate(np.linspace(0, camera.calibration.intrinsics.size.height, rows)):
            for column, i in enumerate(np.linspace(0, camera.calibration.intrinsics.size.width, columns)):
                point = camera.calibration.project_from_image(Point(x=i, y=j))
                if point is not None:
                    camera.projection[row][column] = point.tuple[:2]
