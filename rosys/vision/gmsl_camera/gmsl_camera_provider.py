from __future__ import annotations

import logging
import shutil

from ... import rosys
from ..camera_provider import CameraProvider
from .gmsl_camera import GmslCamera


class GmslCameraProvider(CameraProvider[GmslCamera]):
    """Collects and provides GMSL2/FPD-Link cameras attached to an NVIDIA Jetson.

    Cameras must be added explicitly through ``add_camera()`` (or restored from persistence).
    Auto discovery is not supported.
    """

    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger('rosys.vision.gmsl_camera_provider')
        rosys.on_shutdown(self.shutdown)

    def restore_from_dict(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            self.add_camera(GmslCamera.from_dict(camera_data))

    async def update_device_list(self) -> None:
        pass  # GMSL cameras are registered explicitly; see the class docstring

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('gst-launch-1.0') is not None
