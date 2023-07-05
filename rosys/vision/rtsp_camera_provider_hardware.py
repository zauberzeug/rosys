import asyncio
import io
import logging
import shlex
import subprocess
import sys
from asyncio.subprocess import Process
from typing import Any, Optional

import imgsize
import netifaces as net
import PIL
import requests
from requests.auth import HTTPDigestAuth

import rosys

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


class RtspCameraProviderHardware(CameraProvider):
    """This module collects and provides real RTSP streaming cameras."""

    def __init__(self) -> None:
        super().__init__()

        self.log = logging.getLogger('rosys.rtsp_camera_provider')

        self.last_scan: Optional[float] = None
        self._cameras: dict[str, RtspCamera] = {}
        self._capture_tasks: dict[str, asyncio.Task] = {}
        self._processes: list[Process] = []
        if sys.platform.startswith('darwin'):
            self.arpscan_cmd = 'sudo arp-scan'
        else:
            self.arpscan_cmd = 'sudo /usr/sbin/arp-scan'

        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.update_device_list, 1)
        rosys.on_repeat(lambda: self.prune_images(max_age_seconds=10.0), 5.0)

        self.needs_backup: bool = False
        persistence.register(self)

    @property
    def cameras(self) -> dict[str, RtspCamera]:
        return self._cameras

    def backup(self) -> dict:
        return {'cameras': persistence.to_dict(self._cameras)}

    def restore(self, data: dict[str, Any]) -> None:
        persistence.replace_dict(self._cameras, RtspCamera, data.get('cameras', {}))

    async def capture_images(self, camera: RtspCamera) -> None:
        async def stream():
            # if platform.system() == 'Darwin':
            #     command = f'gst-launch-1.0 rtspsrc location="{camera.url}" latency=0 protocols=tcp drop-on-latency=true buffer-mode=none ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate=6/1" ! jpegenc ! fdsink'
            # else:
            if 'subtype=0' in camera.url:
                url = camera.url.replace('subtype=0', 'subtype=1')
                command = f'gst-launch-1.0 rtspsrc location="{url}" latency=0 protocols=tcp ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate=6/1"! jpegenc ! fdsink'
            else:
                command = f'gst-launch-1.0 rtspsrc location="{camera.url}" latency=0 protocols=tcp ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate=6/1" ! jpegenc ! fdsink'
            self.log.info(command)
            process = await asyncio.create_subprocess_exec(*shlex.split(command), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self._processes.append(process)
            data = b''
            while process.returncode is None:
                new = await process.stdout.read(4096)
                if not new:
                    break
                data += new
                header = 0
                images: list[bytes] = []
                while header < len(data):
                    header = data.find(b'\xff\xd8', header)
                    if header == -1:
                        break
                    footer = data.find(b'\xff\xd9', header)
                    if footer == -1:
                        break
                    images.append(data[header:footer + 2])
                    header = footer + 2
                data = data[header:]
                if images:
                    yield images[-1]
            error = await process.stderr.read()
            self.log.info(f'process {process.pid} exited with {process.returncode} and error {error}')
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
            camera.images.append(Image(camera_id=camera.id, data=image, time=rosys.time(), size=size))
        camera.active = False
        self.invalidate()
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
        for interface in net.interfaces():
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
                await self.activate(camera)

    def get_rtsp_url(self, ip: str, vendor_mac: str) -> Optional[str]:
        if vendor_mac == 'e0:62:90':  # Jovision IP Cameras
            return f'rtsp://admin:admin@{ip}/profile0'
        elif vendor_mac in ['e4:24:6c', '3c:e3:6b']:  # Dahua IP Cameras
            return f'rtsp://admin:Adminadmin@{ip}/cam/realmonitor?channel=1&subtype=0'
        else:
            self.log.debug('ignoring vendor mac {mac} because it seems not to be a known camera')

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await self.deactivate(camera)
        for process in self._processes:
            try:
                process.kill()
            except Exception:
                self.log.info('could not kill process')

    def invalidate(self) -> None:
        self.needs_backup = True

    def get_image_snapshot(self, camera: RtspCamera) -> Optional[Image]:
        if 'profile0' in camera.url:
            return camera.latest_captured_image
        try:
            ip = camera.url.split('@')[1].split('/')[0]
            url = f'http://{ip}/cgi-bin/snapshot.cgi'
            response = requests.get(url, auth=HTTPDigestAuth('admin', 'Adminadmin'))
            image = Image(camera_id=camera.id, data=response.content,
                          time=rosys.time(), size=ImageSize(width=3840, height=2160))
            camera.images.append(image)
            return image
        except:
            return None


def process_image(data: bytes, rotation: ImageRotation, crop: Rectangle = None) -> bytes:
    image = PIL.Image.open(io.BytesIO(data))
    if crop is not None:
        image = image.crop((int(crop.x), int(crop.y), int(crop.x+crop.width), int(crop.y+crop.height)))
    image = image.rotate(int(rotation), expand=True, resample=PIL.Image.Resampling.BICUBIC)
    img_byte_arr = io.BytesIO()
    image.save(img_byte_arr, format='JPEG')
    return img_byte_arr.getvalue()
