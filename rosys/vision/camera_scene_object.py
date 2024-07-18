from typing import Optional

import numpy as np
from nicegui.elements.scene_objects import Cylinder, Group, Line, Text

from . import CalibratableCamera


class CameraSceneObject(Group):

    def __init__(self, camera: CalibratableCamera, color: Optional[str] = None) -> None:
        super().__init__()

        color = color or '#0088ff'

        px_per_m = 1000
        debug = True

        uid = camera.id
        with self.with_name(f'camera_{uid}'):
            with Group() as pyramid:
                Cylinder(0, np.sqrt(0.5), 1, 4, wireframe=True) \
                    .rotate(-np.pi / 2, 0, np.pi / 4) \
                    .move(z=0.5) \
                    .material(self.color)

                if debug:
                    Text(uid)
            pyramid.scale(
                camera.calibration.intrinsics.size.width / px_per_m,
                camera.calibration.intrinsics.size.height / px_per_m,
                camera.calibration.intrinsics.matrix[0][0] / px_per_m,
            )
            with pyramid:
                Line([0, 0, 0], [0.5, 0, 0]).material('#ff0000')
                Line([0, 0, 0], [0, 0.5, 0]).material('#00ff00')
                Line([0, 0, 0], [0, 0, 0.5]).material('#0000ff')
