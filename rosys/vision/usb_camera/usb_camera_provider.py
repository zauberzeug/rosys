import logging
import re
import shutil
from typing import Optional

from ... import persistence, rosys
from ..camera_provider import CameraProvider
from .usb_camera import UsbCamera
from .usb_camera_scanner import scan_for_connected_devices

SCAN_INTERVAL = 10


class UsbCameraProvider(CameraProvider, persistence.PersistentModule):
    """This module collects and provides real USB cameras.

    Camera devices are discovered through video4linux (v4l) and accessed with openCV.
    Therefore the program v4l2ctl and openCV (including python bindings) must be available.
    """

    def __init__(self) -> None:
        super().__init__()

        self.log = logging.getLogger('rosys.usb_camera_provider')

        self.last_scan: Optional[float] = None
        self._cameras: dict[str, UsbCamera] = {}

        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.update_device_list, 1)

    def backup(self) -> dict:
        cameras = {}
        for camera in self._cameras.values():
            cameras[camera.id] = camera.to_dict()
        return {'cameras': cameras}

    def restore(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            camera = UsbCamera.from_dict(camera_data)
            self.add_camera(camera)

        for camera in self._cameras.values():
            camera.NEW_IMAGE.register(self.NEW_IMAGE.emit)

    @staticmethod
    async def scan_for_cameras() -> list[str]:
        uids = scan_for_connected_devices()

        return list(uids)

    async def update_device_list(self) -> None:
        if self.last_scan is not None and rosys.time() < self.last_scan + SCAN_INTERVAL:
            return
        self.last_scan = rosys.time()
        camera_uids = await UsbCameraProvider.scan_for_cameras()
        for uid in camera_uids:
            if not uid in self._cameras:
                self.add_camera(UsbCamera(id=uid))
            elif not self._cameras[uid].is_connected:
                await self._cameras[uid].connect()

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('v4l2-ctl') is not None
