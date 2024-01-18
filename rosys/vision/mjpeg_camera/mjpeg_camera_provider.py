import logging
from typing import Optional

from ... import persistence, rosys
from ..camera_provider import CameraProvider
from ..rtsp_camera.arp_scan import find_cameras
from .mjpeg_camera import MjpegCamera
from .vendors import VendorType, mac_to_vendor


class MjpegCameraProvider(CameraProvider[MjpegCamera], persistence.PersistentModule):

    def __init__(self, username: Optional[str] = None, password: Optional[str] = None) -> None:
        super().__init__()

        self.username = username
        self.password = password

        self.log = logging.getLogger('rosys.mjpeg_camera_provider')
        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.update_device_list, 5.)

    def restore(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            camera_data['password'] = self.password
            camera_data['username'] = self.username
            camera = MjpegCamera.from_dict(camera_data)
            self.add_camera(camera)
        for camera in self._cameras.values():
            camera.NEW_IMAGE.register(self.NEW_IMAGE.emit)

    @staticmethod
    async def scan_for_cameras() -> list[str]:
        return [mac async for mac, _ in find_cameras() if mac_to_vendor(mac) == VendorType.AXIS]

    async def update_device_list(self) -> None:
        newly_disconnected_cameras = {id for id, camera in self._cameras.items() if camera.is_connected}
        for mac in await self.scan_for_cameras():
            if mac not in self._cameras:
                self.add_camera(MjpegCamera(id=mac, username=self.username, password=self.password))
            if mac in newly_disconnected_cameras:
                newly_disconnected_cameras.remove(mac)
            camera = self._cameras[mac]
            if not camera.is_connected:
                self.log.info(f'activating authorized camera {camera.id}...')
                await camera.connect()

        for mac in newly_disconnected_cameras:
            await self._cameras[mac].disconnect()

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()
