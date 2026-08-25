import logging
from typing import Literal

from ... import rosys
from ..camera_provider import CameraProvider
from .arp_scan import find_known_cameras
from .rtsp_camera import RtspCamera


class RtspCameraProvider(CameraProvider[RtspCamera]):
    """This module collects and provides real RTSP streaming cameras."""
    SCAN_INTERVAL = 10

    def __init__(self, *,
                 frame_rate: int = 6,
                 substream: int = 0,
                 avdec: Literal['h264', 'h265'] = 'h264',
                 network_interface: str | None = None,
                 auto_scan: bool = True) -> None:
        super().__init__()

        self.frame_rate = frame_rate
        self.substream = substream
        self.network_interface = network_interface
        self.avdec: Literal['h264', 'h265'] = avdec

        self.log = logging.getLogger('rosys.rtsp_camera_provider')

        rosys.on_shutdown(self.shutdown)
        if auto_scan:
            rosys.on_repeat(self.update_device_list, self.SCAN_INTERVAL)

    def restore_from_dict(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            self.add_camera(RtspCamera.from_dict(camera_data))

    @staticmethod
    async def scan_for_cameras(network_interface: str | None = None) -> list[tuple[str, str]]:
        return await find_known_cameras(network_interface=network_interface)

    async def update_device_list(self) -> None:
        self.log.debug('scanning for cameras...')
        for mac, ip in await find_known_cameras(network_interface=self.network_interface):
            camera = next((c for c in self._cameras.values() if c.mac == mac), None)
            if camera is None:
                self.log.debug('found new camera %s', mac)
                self.add_camera(RtspCamera(mac=mac, fps=self.frame_rate,
                                           substream=self.substream, avdec=self.avdec, ip=ip))
            else:
                camera.ip = ip

        self.log.debug('scanning completed, found %d cameras', len(self._cameras))

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()
