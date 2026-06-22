from __future__ import annotations

import logging
import shutil

from ... import rosys
from ..camera_provider import CameraProvider
from .gmsl_camera import GmslCamera
from .gmsl_camera_scanner import scan_for_sensor_ids


class GmslCameraProvider(CameraProvider[GmslCamera]):
    """Collects and provides GMSL2/FPD-Link cameras attached to an NVIDIA Jetson.

    Cameras are accessed through NVIDIA's Argus stack (`nvarguscamerasrc`). Discovery via
    `v4l2-ctl --list-devices` is best-effort: the mapping from `/dev/videoN` to an Argus
    ``sensor-id`` must be confirmed on the target board, so ``auto_scan`` defaults to ``False``
    and cameras are usually registered explicitly with a known ``sensor_id``.
    """
    SCAN_INTERVAL = 10

    def __init__(self, *, auto_scan: bool = False) -> None:
        super().__init__()
        self.log = logging.getLogger('rosys.vision.gmsl_camera_provider')
        rosys.on_shutdown(self.shutdown)
        if auto_scan:
            rosys.on_repeat(self.update_device_list, self.SCAN_INTERVAL)

    def restore_from_dict(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            self.add_camera(GmslCamera.from_dict(camera_data))

    @staticmethod
    async def scan_for_cameras() -> set[int]:
        return await rosys.run.io_bound(scan_for_sensor_ids) or set()

    async def update_device_list(self) -> None:
        for sensor_id in await self.scan_for_cameras():
            camera_id = f'gmsl-{sensor_id}'
            if camera_id not in self._cameras:
                self.add_camera(GmslCamera(id=camera_id, sensor_id=sensor_id))
            await self._cameras[camera_id].connect()

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('gst-launch-1.0') is not None
