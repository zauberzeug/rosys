import io
import logging
import sys
from typing import Any, Optional

import netifaces
import requests
from requests.auth import HTTPDigestAuth

from .. import persistence, rosys
from .camera_provider import CameraProvider
from .image import Image, ImageSize
from .rtsp_camera import RtspCamera

SCAN_INTERVAL = 10


class RtspCameraProvider(CameraProvider):
    """This module collects and provides real RTSP streaming cameras."""
    USE_PERSISTENCE: bool = False

    def __init__(self, *, frame_rate: int = 6, jovision_profile: int = 0) -> None:
        super().__init__()

        self.frame_rate = frame_rate
        self.jovision_profile = jovision_profile

        self.log = logging.getLogger('rosys.rtsp_camera_provider')

        self.last_scan: Optional[float] = None
        self._cameras: dict[str, RtspCamera] = {}

        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.update_device_list, 1)

        self.needs_backup: bool = False
        if self.USE_PERSISTENCE:
            persistence.register(self)

    def backup(self) -> dict:
        return {'cameras': persistence.to_dict(self._cameras)}

    def restore(self, data: dict[str, dict]) -> None:
        persistence.replace_dict(self._cameras, RtspCamera, data.get('cameras', {}))
        for camera in self._cameras.values():
            camera.NEW_IMAGE.register(self.NEW_IMAGE.emit)

    @staticmethod
    async def run_arp_scan(interface) -> Optional[str]:
        if sys.platform.startswith('darwin'):
            arpscan_cmd = 'sudo arp-scan'
        else:
            arpscan_cmd = 'sudo /usr/sbin/arp-scan'  # TODO is this necessary? sbin should be in path

        cmd = f'{arpscan_cmd} -I {interface} --localnet'
        output = await rosys.run.sh(cmd, timeout=10)
        if output is None:
            return 'ERROR'
        if 'sudo' in output:
            print(output)
            raise Exception('Could not run arp-scan! Make sure it is installed can be run with sudo.'
                            'Try running sudo visudo and add the following line: "rosys ALL=(ALL) NOPASSWD: /usr/sbin/arp-scan"')

        return output

    @staticmethod
    async def scan_for_cameras() -> None:
        cameras_ids = []
        for interface in netifaces.interfaces():
            output = await RtspCameraProvider.run_arp_scan(interface)
            if 'ERROR' in output:
                continue
            for line in output.splitlines():
                infos = line.split()
                if len(infos) < 2:
                    continue
                try:
                    ip, mac = infos[:2]
                except Exception:
                    logging.exception(f'could not parse {line}')
                    continue
                if not RtspCamera.known_vendor(mac):
                    continue
                cameras_ids.append(mac)

        return cameras_ids

    async def update_device_list(self) -> None:
        if self.last_scan is not None and rosys.time() < self.last_scan + SCAN_INTERVAL:
            return
        self.last_scan = rosys.time()
        camera_ids = await self.scan_for_cameras()
        for mac in camera_ids:
            if mac not in self._cameras:
                self.add_camera(RtspCamera(id=mac, goal_fps=self.frame_rate, jovision_profile=self.jovision_profile))
            camera = self._cameras[mac]
            if not camera.is_connected:
                self.log.info(f'activating authorized camera {camera.id}...')
                await camera.connect()
                print(f'activated camera {camera.id}', flush=True)

    def get_rtsp_url(self, ip: str, vendor_mac: str) -> Optional[str]:
        if vendor_mac == 'e0:62:90':  # Jovision IP Cameras
            return f'rtsp://admin:admin@{ip}/profile{self.jovision_profile}'
        if vendor_mac in ['e4:24:6c', '3c:e3:6b']:  # Dahua IP Cameras
            return f'rtsp://admin:Adminadmin@{ip}/cam/realmonitor?channel=1&subtype=0'
        logging.debug(f'ignoring vendor mac {vendor_mac} because it seems not to be a known camera')
        return None

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()

    def invalidate(self) -> None:
        self.needs_backup = True

    def get_image_snapshot(self, camera: RtspCamera) -> Optional[Image]:
        assert camera.url is not None
        if 'profile0' in camera.url:
            return camera.latest_captured_image
        try:
            ip = camera.url.split('@')[1].split('/')[0]
            url = f'http://{ip}/cgi-bin/snapshot.cgi'
            response = requests.get(url, auth=HTTPDigestAuth('admin', 'Adminadmin'), timeout=3.0)
            if response.status_code != 200:
                self.log.warning(f'could not get snapshot from {url}, status code {response.status_code}:\n'
                                 f'{response.content.decode()}')
                return None
            image = Image(camera_id=camera.id, data=response.content,
                          time=rosys.time(), size=ImageSize(width=3840, height=2160))
            camera.images.append(image)
            return image
        except Exception:
            return None