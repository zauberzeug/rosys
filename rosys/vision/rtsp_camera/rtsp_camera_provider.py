import logging

from ... import rosys
from ...helpers.deprecation import deprecated_param
from ..camera_provider import CameraProvider
from .arp_scan import find_known_cameras
from .rtsp_camera import RtspCamera


class RtspCameraProvider(CameraProvider[RtspCamera]):
    """This module collects and provides real RTSP streaming cameras."""
    SCAN_INTERVAL = 10

    @deprecated_param('jovision_profile', remove_in_version='0.27.0')
    def __init__(self, *,
                 frame_rate: int = 6,
                 substream: int = 0,
                 jovision_profile: int | None = None,
                 network_interface: str | None = None,
                 auto_scan: bool = True) -> None:
        super().__init__()

        self.frame_rate = frame_rate
        self.substream = jovision_profile if jovision_profile is not None else substream
        self.network_interface = network_interface

        self.log = logging.getLogger('rosys.rtsp_camera_provider')

        rosys.on_shutdown(self.shutdown)
        if auto_scan:
            rosys.on_repeat(self.update_device_list, self.SCAN_INTERVAL)

    def backup_to_dict(self) -> dict:
        cameras = {}
        for camera in self._cameras.values():
            cameras[camera.id] = camera.to_dict()
        return {'cameras': cameras}

    def restore_from_dict(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            camera = RtspCamera.from_dict(camera_data)
            self.add_camera(camera)
        for camera in self._cameras.values():
            camera.NEW_IMAGE.register(self.NEW_IMAGE.emit)

    @staticmethod
    async def scan_for_cameras(network_interface: str | None = None) -> list[tuple[str, str]]:
        return await find_known_cameras(network_interface=network_interface)

    async def update_device_list(self) -> None:
        self.log.debug('scanning for cameras...')
        for mac, ip in await find_known_cameras(network_interface=self.network_interface):
            if mac not in self._cameras:
                self.log.debug('found new camera %s', mac)
                self.add_camera(RtspCamera(id=mac, fps=self.frame_rate, substream=self.substream, ip=ip))
            camera = self._cameras[mac]
            if not camera.is_connected:
                self.log.info('activating authorized camera %s...', camera.id)
                camera.ip = ip
                await camera.connect()

        self.log.debug('scanning completed, found %d cameras', len(self._cameras))

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()
