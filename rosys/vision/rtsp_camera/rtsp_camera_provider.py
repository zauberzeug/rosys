import logging
import os
import sys
from typing import Optional

import netifaces
import requests
from requests.auth import HTTPDigestAuth

from ... import persistence, rosys
from ..camera_provider import CameraProvider
from ..image import Image, ImageSize
from . import RtspCamera

SCAN_INTERVAL = 10


class RtspCameraProvider(CameraProvider, persistence.PersistentModule):
    """This module collects and provides real RTSP streaming cameras."""

    def __init__(self, *, frame_rate: int = 6, jovision_profile: int = 0) -> None:
        super().__init__()

        self.frame_rate = frame_rate
        self.jovision_profile = jovision_profile

        self.log = logging.getLogger('rosys.rtsp_camera_provider')

        self.last_scan: Optional[float] = None
        self._cameras: dict[str, RtspCamera] = {}

        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.update_device_list, 10.)

    def backup(self) -> dict:
        cameras = {}
        for camera in self._cameras.values():
            cameras[camera.id] = camera.__to_dict__()
        return {'cameras': cameras}

    def restore(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            camera = RtspCamera.from_dict(camera_data)
            self.add_camera(camera)
        for camera in self._cameras.values():
            camera.NEW_IMAGE.register(self.NEW_IMAGE.emit)

    @staticmethod
    async def run_arp_scan(interface) -> str:
        if sys.platform.startswith('darwin'):
            arpscan_cmd = 'arp-scan'
        else:
            arpscan_cmd = '/usr/sbin/arp-scan'  # TODO is this necessary? sbin should be in path
        if os.getuid() != 0:
            arpscan_cmd = f'sudo {arpscan_cmd}'

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
    async def scan_for_cameras() -> list[str]:
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
                    mac = infos[1]
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
        newly_disconnected_cameras = set([camera.id for camera in self._cameras.values() if camera.is_connected])
        for mac in camera_ids:
            if mac not in self._cameras:
                self.add_camera(RtspCamera(id=mac, goal_fps=self.frame_rate, jovision_profile=self.jovision_profile))
            if mac in newly_disconnected_cameras:
                newly_disconnected_cameras.remove(mac)
            camera = self._cameras[mac]
            if not camera.is_connected:
                self.log.info(f'activating authorized camera {camera.id}...')
                await camera.connect()
                print(f'activated camera {camera.id}', flush=True)

        for mac in newly_disconnected_cameras:
            print(f'disconnecting camera {mac} since it cannot be found anymore', flush=True)
            await self._cameras[mac].disconnect()

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()

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
