import logging
from typing import Optional

import httpx

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

    async def scan_for_cameras(self) -> list[tuple[str, str]]:
        ids_ips: list[tuple[str, str]] = []
        async for mac, ip in find_cameras():
            vendor = mac_to_vendor(mac)
            if vendor == VendorType.OTHER:
                continue

            if vendor == VendorType.AXIS:
                authentication = None if self.username is None or self.password is None else \
                    httpx.DigestAuth(self.username, self.password)
                url = f'http://{ip}/axis-cgi/videostatus.cgi'
                try:
                    async with httpx.AsyncClient() as client:
                        response = await client.get(url, auth=authentication)
                except httpx.HTTPError as e:
                    self.log.warning('Error while looking for cameras at axis router (%s): %s', url, e)
                    continue
                if response.status_code != 200:
                    continue

                camera_infos = response.text.split('\n')
                for info_line in camera_infos:
                    if info_line.endswith(' = video'):
                        video_name = info_line.removesuffix(' = video')
                        index = video_name.split(' ')[1]
                        ids_ips.append((f'{mac}-{index}', ip))
            else:
                ids_ips.append((mac, ip))
        return ids_ips

    async def update_device_list(self) -> None:
        newly_disconnected_cameras = {id for id, camera in self._cameras.items() if camera.is_connected}
        for camera_id, ip in await self.scan_for_cameras():
            if camera_id not in self._cameras:
                self.add_camera(MjpegCamera(id=camera_id, username=self.username,
                                password=self.password, connect_after_init=False))
            if camera_id in newly_disconnected_cameras:
                newly_disconnected_cameras.remove(camera_id)
            camera = self._cameras[camera_id]
            if not camera.is_connected:
                self.log.info('activating authorized camera "%s" at ip "%s" ...', camera.id, ip)
                await camera.connect(ip=ip)

        for mac in newly_disconnected_cameras:
            await self._cameras[mac].disconnect()

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()
