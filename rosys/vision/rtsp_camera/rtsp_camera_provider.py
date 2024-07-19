import logging

from ... import persistence, rosys
from ..camera_provider import CameraProvider
from .arp_scan import find_known_cameras
from .rtsp_camera import RtspCamera


class RtspCameraProvider(CameraProvider[RtspCamera], persistence.PersistentModule):
    """This module collects and provides real RTSP streaming cameras."""
    SCAN_INTERVAL = 10

    def __init__(self, *,
                 frame_rate: int = 6,
                 jovision_profile: int = 0,
                 network_interface: str | None = None,
                 auto_scan: bool = True) -> None:
        super().__init__()

        self.frame_rate = frame_rate
        self.jovision_profile = jovision_profile
        self.network_interface = network_interface

        self.log = logging.getLogger('rosys.rtsp_camera_provider')

        rosys.on_shutdown(self.shutdown)
        if auto_scan:
            rosys.on_repeat(self.update_device_list, self.SCAN_INTERVAL)

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
    async def scan_for_cameras(network_interface: str | None = None) -> list[tuple[str, str]]:
        return await find_known_cameras(network_interface=network_interface)

    async def update_device_list(self) -> None:
        for mac, ip in await find_known_cameras(network_interface=self.network_interface):
            if mac not in self._cameras:
                self.add_camera(RtspCamera(id=mac, fps=self.frame_rate, jovision_profile=self.jovision_profile, ip=ip))
            camera = self._cameras[mac]
            if not camera.is_connected:
                self.log.info('activating authorized camera %s...', camera.id)
                camera.ip = ip
                await camera.connect()

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()
