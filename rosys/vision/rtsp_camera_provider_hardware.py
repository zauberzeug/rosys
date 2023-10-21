import asyncio
import io
import logging
import os
import shlex
import subprocess
import sys
from asyncio.subprocess import Process
from io import BytesIO
from typing import Any, AsyncGenerator, Optional

import imgsize
import netifaces
import PIL
import requests
from requests.auth import HTTPDigestAuth

from .. import persistence, rosys
from ..geometry import Rectangle
from .camera_provider import CameraProvider
from .image import Image, ImageSize
from .rtsp_camera import ImageRotation, RtspCamera

SCAN_INTERVAL = 10


class PeekableBytesIO(io.BytesIO):

    def peek(self, n=-1):
        position = self.tell()
        data = self.read(n)
        self.seek(position)
        return data


class RtspCameraProviderHardware(CameraProvider, persistence.PersistentModule):
    """This module collects and provides real RTSP streaming cameras."""

    def __init__(self, *, frame_rate: int = 6, jovision_profile: int = 0) -> None:
        super().__init__()

        self.frame_rate = frame_rate
        self.jovision_profile = jovision_profile

        self.log = logging.getLogger('rosys.rtsp_camera_provider')

        self.last_scan: Optional[float] = None
        self._cameras: dict[str, RtspCamera] = {}
        self._capture_tasks: dict[str, asyncio.Task] = {}
        self._processes: list[Process] = []

        if sys.platform.startswith('darwin'):
            self.arpscan_cmd = 'arp-scan'
        else:
            self.arpscan_cmd = '/usr/sbin/arp-scan'
        if os.getuid() != 0:
            self.arpscan_cmd = f'sudo {self.arpscan_cmd}'

        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.update_device_list, 1)
        rosys.on_repeat(lambda: self.prune_images(max_age_seconds=10.0), 5.0)

    @property
    def cameras(self) -> dict[str, RtspCamera]:
        return self._cameras

    def backup(self) -> dict:
        return {'cameras': persistence.to_dict(self._cameras)}

    def restore(self, data: dict[str, Any]) -> None:
        persistence.replace_dict(self._cameras, RtspCamera, data.get('cameras', {}))

    async def capture_images(self, camera: RtspCamera) -> None:
        async def stream() -> AsyncGenerator[bytes, None]:
            assert camera.url is not None
            # if platform.system() == 'Darwin':
            #     command = f'gst-launch-1.0 rtspsrc location="{camera.url}" latency=0 protocols=tcp drop-on-latency=true buffer-mode=none ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate=6/1" ! jpegenc ! fdsink'
            # else:
            logging.info(f'capture images from {camera.id} with URL {camera.url}')
            assert camera.url is not None
            if 'subtype=0' in camera.url:
                # to try: replace avdec_h264 with nvh264dec ! nvvidconv (!videoconvert)
                url = camera.url.replace('subtype=0', 'subtype=1')
                command = f'gst-launch-1.0 rtspsrc location="{url}" latency=0 protocols=tcp ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate={self.frame_rate}/1"! jpegenc ! fdsink'
            else:
                command = f'gst-launch-1.0 rtspsrc location="{camera.url}" latency=0 protocols=tcp ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate={self.frame_rate}/1" ! jpegenc ! fdsink'
            process = await asyncio.create_subprocess_exec(*shlex.split(command), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            assert process.stdout is not None
            assert process.stderr is not None
            self._processes.append(process)

            buffer = BytesIO()
            pos = 0
            header = None

            while process.returncode is None:
                assert process.stdout is not None
                new = await process.stdout.read(4096)
                if not new:
                    break
                buffer.write(new)

                img_range = None
                while True:
                    if header is None:
                        h = buffer.getvalue().find(b'\xff\xd8', pos)
                        if h == -1:
                            pos = buffer.tell() - 1
                            break
                        pos = h + 2
                        header = h
                    else:
                        f = buffer.getvalue().find(b'\xff\xd9', pos)
                        if f == -1:
                            pos = buffer.tell() - 1
                            break
                        img_range = (header, f + 2)
                        pos = f + 2
                        header = None

                if img_range:
                    yield buffer.getvalue()[img_range[0]:img_range[1]]

                    rest = buffer.getvalue()[img_range[1]:]
                    buffer.seek(0)
                    buffer.truncate()
                    buffer.write(rest)

                    pos = 0
                    if header is not None:
                        header -= img_range[1]

            assert process.stderr is not None
            error = await process.stderr.read()
            self.log.info(f'process {process.pid} exited with {process.returncode} and error {error.decode()}')
            if 'Unauthorized' in error.decode():
                camera.authorized = False

        async for image in stream():
            camera.active = True
            if camera.crop or camera.rotation != ImageRotation.NONE:
                image = await rosys.run.cpu_bound(process_image, image, camera.rotation, camera.crop)
            try:
                with PeekableBytesIO(image) as f:
                    width, height = imgsize.get_size(f)
            except imgsize.UnknownSize:
                continue
            size = ImageSize(width=width, height=height)
            camera.resolution = size
            self.add_image(camera, Image(camera_id=camera.id, data=image, time=rosys.time(), size=size))
        camera.active = False
        self.request_backup()
        self._capture_tasks.pop(camera.id)

    async def activate(self, camera: RtspCamera) -> None:
        task = rosys.background_tasks.create(self.capture_images(camera), name=f'capture {camera.id}')
        if task is None:
            self.log.warning(f'could not create task for {camera.id}')
            return
        self._capture_tasks[camera.id] = task
        await self.CAMERA_ADDED.call(camera)

    async def deactivate(self, camera: RtspCamera) -> None:
        if camera.id not in self._capture_tasks:
            return
        self._capture_tasks.pop(camera.id).cancel()

    async def update_device_list(self) -> None:
        if self.last_scan is not None and rosys.time() < self.last_scan + SCAN_INTERVAL:
            return
        self.last_scan = rosys.time()
        old_cameras = [camera for camera in self._cameras.values() if camera.active]
        for interface in netifaces.interfaces():
            cmd = f'{self.arpscan_cmd} -I {interface} --localnet'
            output = await rosys.run.sh(cmd, timeout=10)
            if output is None or 'ERROR' in output:
                continue
            if 'sudo' in output:
                self.log.error('could not run arp-scan, try running "sudo visudo" '
                               'and add the following line: "rosys ALL=(ALL) NOPASSWD: /usr/sbin/arp-scan"')
                return
            for line in output.splitlines():
                infos = line.split()
                if len(infos) < 2:
                    continue
                try:
                    ip, mac = infos[:2]
                except Exception:
                    self.log.exception(f'could not parse {line}')
                    continue
                url = self.get_rtsp_url(ip, mac[:8])
                if url is None:
                    continue
                if mac not in self._cameras:
                    camera = RtspCamera(id=mac, url=url)
                    self._cameras[mac] = camera
                    self.log.info(f'adding camera {url}')
                    await self.CAMERA_ADDED.call(camera)
                camera = self._cameras[mac]
                camera.url = url
                if self._cameras[mac] in old_cameras:
                    old_cameras.remove(self._cameras[mac])
        for camera in old_cameras:
            camera.active = False
        for camera in self._cameras.values():
            if camera.id not in self._capture_tasks and camera.authorized:
                self.log.info(f'activating authorized camera {camera.id} with url {camera.url}...')
                await self.activate(camera)

    def get_rtsp_url(self, ip: str, vendor_mac: str) -> Optional[str]:
        if vendor_mac == 'e0:62:90':  # Jovision IP Cameras
            return f'rtsp://admin:admin@{ip}/profile{self.jovision_profile}'
        if vendor_mac in ['e4:24:6c', '3c:e3:6b']:  # Dahua IP Cameras
            return f'rtsp://admin:Adminadmin@{ip}/cam/realmonitor?channel=1&subtype=0'
        self.log.debug(f'ignoring vendor mac {vendor_mac} because it seems not to be a known camera')
        return None

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await self.deactivate(camera)
        for process in self._processes:
            try:
                process.kill()
            except Exception:
                self.log.info('could not kill process')

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


def process_image(data: bytes, rotation: ImageRotation, crop: Optional[Rectangle] = None) -> bytes:
    image = PIL.Image.open(io.BytesIO(data))
    if crop is not None:
        image = image.crop((int(crop.x), int(crop.y), int(crop.x + crop.width), int(crop.y + crop.height)))
    image = image.rotate(int(rotation), expand=True)  # NOTE: PIL handles rotation with 90 degree steps efficiently
    img_byte_arr = io.BytesIO()
    image.save(img_byte_arr, format='JPEG')
    return img_byte_arr.getvalue()
