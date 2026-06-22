from __future__ import annotations

import logging
import shutil

from ... import rosys
from ..camera_provider import CameraProvider
from .gmsl_camera import GmslCamera


class GmslCameraProvider(CameraProvider[GmslCamera]):
    """Collects and provides GMSL2/FPD-Link cameras attached to an NVIDIA Jetson.

    Cameras are accessed through NVIDIA's Argus stack (`nvarguscamerasrc`) and must be
    registered explicitly with their Argus ``sensor_id`` (the GMSL port), e.g.::

        provider.add_camera(GmslCamera(id='gmsl-0', sensor_id=0))

    There is no automatic discovery: whether a port has a camera connected cannot be told
    from Video4Linux or sysfs (the device tree exposes every defined port as a `/dev/video`
    node regardless), and the only reliable signals -- an Argus capture attempt or the GMSL
    deserializer link-lock register over I2C -- are too expensive or too board-specific to
    run as a generic scan. The ``sensor_id`` maps 1:1 to ``/dev/video<sensor_id>`` and to the
    device-tree camera-module index.
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
