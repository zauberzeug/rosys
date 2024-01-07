import logging
from typing import Optional

from ... import persistence, rosys
from ..camera_provider import CameraProvider
from .arp_scan import find_known_cameras
from .rtsp_camera import RtspCamera

SCAN_INTERVAL = 10


class RtspCameraProvider(CameraProvider[RtspCamera], persistence.PersistentModule):
    """This module collects and provides real RTSP streaming cameras."""

    def __init__(self, *, frame_rate: int = 6, jovision_profile: int = 0) -> None:
        super().__init__()

        self.frame_rate = frame_rate
        self.jovision_profile = jovision_profile

        self.log = logging.getLogger('rosys.rtsp_camera_provider')

        self.last_scan: Optional[float] = None

        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.update_device_list, 10.)

    def backup(self) -> dict:
        cameras = {}
        for camera in self._cameras.values():
            cameras[camera.id] = camera.to_dict()
        return {'cameras': cameras}

    def restore(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            camera = RtspCamera.from_dict(camera_data)
            self.add_camera(camera)
        for camera in self._cameras.values():
            camera.NEW_IMAGE.register(self.NEW_IMAGE.emit)

    @staticmethod
    async def scan_for_cameras() -> list[str]:
        return await find_known_cameras()

    async def update_device_list(self) -> None:
        if self.last_scan is not None and rosys.time() < self.last_scan + SCAN_INTERVAL:
            return
        self.last_scan = rosys.time()
        newly_disconnected_cameras = {id for id, camera in self._cameras.items() if camera.is_connected}
        for mac in await find_known_cameras():
            if mac not in self._cameras:
                self.add_camera(RtspCamera(id=mac, goal_fps=self.frame_rate, jovision_profile=self.jovision_profile))
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
