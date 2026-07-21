import logging
import shutil

from ... import rosys
from ..camera_provider import CameraProvider
from .usb_camera import UsbCamera
from .usb_camera_scanner import scan_for_connected_devices


class UsbCameraProvider(CameraProvider[UsbCamera]):
    """This module collects and provides real USB cameras.

    Camera devices are discovered through video4linux (v4l) and accessed with openCV.
    Therefore the program v4l2ctl and openCV (including python bindings) must be available.
    """
    SCAN_INTERVAL = 10

    def __init__(self, *, auto_scan: bool = True) -> None:
        super().__init__()

        self.log = logging.getLogger('rosys.usb_camera_provider')

        rosys.on_shutdown(self.shutdown)
        if auto_scan:
            rosys.on_repeat(self.update_device_list, self.SCAN_INTERVAL)

    def backup_to_dict(self) -> dict:
        return {
            'cameras': {camera.id: camera.to_dict() for camera in self._cameras.values()}
        }

    def restore_from_dict(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            self.add_camera(UsbCamera.from_dict(camera_data))

    @staticmethod
    async def scan_for_cameras() -> set[str]:
        return (await rosys.run.io_bound(scan_for_connected_devices)) or set()

    async def update_device_list(self) -> None:
        camera_uids = await self.scan_for_cameras()
        for uid in camera_uids:
            if uid not in self._cameras:
                self.add_camera(UsbCamera(id=uid))
            await self._cameras[uid].connect()

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('v4l2-ctl') is not None
