import logging

from ... import persistence, rosys
from ..camera_provider import CameraProvider
from ..rtsp_camera.arp_scan import find_cameras
from ..rtsp_camera.vendors import VendorType, mac_to_vendor
from .mjpeg_camera import MjpegCamera


class MjpegCameraProvider(CameraProvider[MjpegCamera], persistence.PersistentModule):

    def __init__(self) -> None:
        super().__init__()

        self.log = logging.getLogger('rosys.mjpeg_camera_provider')
        rosys.on_repeat(self.update_device_list, 5.)

    def restore(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            camera = MjpegCamera.from_dict(camera_data)
            self.add_camera(camera)
        for camera in self._cameras.values():
            camera.NEW_IMAGE.register(self.NEW_IMAGE.emit)

    async def scan_for_cameras(self) -> list[str]:
        cam_list = [mac async for mac, _ in find_cameras() if mac_to_vendor(mac) == VendorType.AXIS]
        print(cam_list)
        return cam_list

    async def update_device_list(self) -> None:
        newly_disconnected_cameras = {id for id, camera in self._cameras.items() if camera.is_connected}
        for mac in await self.scan_for_cameras():
            if mac not in self._cameras:
                self.add_camera(MjpegCamera(id=mac))
            if mac in newly_disconnected_cameras:
                newly_disconnected_cameras.remove(mac)
            camera = self._cameras[mac]
            if not camera.is_connected:
                self.log.info(f'activating authorized camera {camera.id}...')
                await camera.connect()

        for mac in newly_disconnected_cameras:
            await self._cameras[mac].disconnect()
