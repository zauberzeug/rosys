import numpy as np
from nicegui import ui

from .camera import CalibratableCamera


class CameraSceneObject(ui.scene.group):

    def __init__(self,
                 camera: CalibratableCamera, *,
                 color: str = '#333333',
                 px_per_m: float = 1000,
                 show_name: bool = True,
                 show_axes: bool = True,
                 ) -> None:
        super().__init__()
        with self:
            with ui.scene.group() as pyramid:
                ui.scene.cylinder(0, np.sqrt(0.5), 1, 4, wireframe=True) \
                    .rotate(-np.pi / 2, 0, np.pi / 4) \
                    .move(z=0.5) \
                    .material(color)
                if show_axes:
                    ui.scene.line([0, 0, 0], [0.5, 0, 0]).material('#ff0000')
                    ui.scene.line([0, 0, 0], [0, 0.5, 0]).material('#00ff00')
                    ui.scene.line([0, 0, 0], [0, 0, 0.5]).material('#0000ff')
                if show_name:
                    ui.scene.text(camera.name)
            if camera.calibration is not None:
                pyramid.scale(
                    camera.calibration.intrinsics.size.width / px_per_m,
                    camera.calibration.intrinsics.size.height / px_per_m,
                    camera.calibration.intrinsics.matrix[0][0] / px_per_m,
                )
