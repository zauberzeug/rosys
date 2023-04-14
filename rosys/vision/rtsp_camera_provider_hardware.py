#!
import asyncio
import hashlib
import io
import logging
import platform
import re
import shlex
import shutil
import subprocess
import sys
from asyncio.subprocess import Process
from dataclasses import dataclass, field
from typing import Any, Optional

import cv2
import netifaces as net
import numpy as np
import PIL

import rosys

from .. import persistence, rosys
from ..geometry import Rectangle
from ..helpers import measure
from .camera_provider import CameraProvider
from .image import Image, ImageSize
from .rtsp_camera import ImageRotation, RtspCamera

SCAN_INTERVAL = 10


class RtspCameraProviderHardware(CameraProvider):
    '''This module collects and provides real RTSP streaming cameras.
    '''

    def __init__(self) -> None:
        super().__init__()

        self.log = logging.getLogger('rosys.rtsp_camera_provider')

        self.last_scan: Optional[float] = None
        self._cameras: dict[str, RtspCamera] = {}
        self._capture_tasks: dict[str, asyncio.Task] = {}
        self._processes: list[Process] = []
        if sys.platform.startswith('darwin'):
            self.arpscan_cmd = 'arp-scan'
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
            if platform.system() == 'Darwin':
                command = f'gst-launch-1.0 rtspsrc location={camera.url} latency=0 protocols=tcp drop-on-latency=true buffer-mode=none ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate=6/1" ! jpegenc ! fdsink'
            else:
                command = f'gst-launch-1.0 rtspsrc location={camera.url} latency=0 protocols=tcp ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate=3/1" ! jpegenc ! fdsink'
            self.log.info(command)
            process = await asyncio.create_subprocess_exec(*shlex.split(command), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self._processes.append(process)
            data = b''
            while True:
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

        size = ImageSize(width=camera.resolution.width, height=camera.resolution.height)
        async for image in stream():
            if camera.crop or camera.rotation != ImageRotation.NONE:
                image = await rosys.run.cpu_bound(process_image, image, camera.rotation, camera.crop)
            camera.images.append(Image(camera_id=camera.id, data=image, time=rosys.time(), size=size))

    async def activate(self, camera: RtspCamera) -> None:
        task = rosys.background_tasks.create(self.capture_images(camera), name=f'capture {camera.id}')
        if task is None:
            self.log.info(f'could not create task for {camera.id}')
            return
        self._capture_tasks[camera.id] = task
        self.log.info(f'activated {camera.id}')
        await self.CAMERA_ADDED.call(camera)

    async def deactivate(self, camera: RtspCamera) -> None:
        self.log.info(f'deactivating {camera.id}')
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
            output = (await rosys.run.sh(cmd, timeout=2))
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
                if not mac.startswith('e0:62:90'):
                    # self.log.debug('ignoring mac {mac} because it seems not to be a camera')
                    continue
                url = f'rtsp://admin:admin@{ip}/profile0'
                if mac not in self._cameras:
                    camera = RtspCamera(id=mac, url=url)
                    self._cameras[mac] = camera
                    self.log.info(f'adding camera {url}')
                    await self.CAMERA_ADDED.call(camera)
                camera = self._cameras[mac]
                camera.url = url
                camera.active = True
                camera.resolution = ImageSize(width=1920, height=1080)  # TODO determine from image stream
                if self._cameras[mac] in old_cameras:
                    old_cameras.remove(self._cameras[mac])
        # for camera in old_cameras:
        #     camera.active = False
        for camera in self._cameras.values():
            if camera.active and camera.id not in self._capture_tasks:
                await self.activate(camera)
            # NOTE deactivating is still buggy -- disabled for now
            # if not camera.active and camera.id in self._capture_tasks:
            #     await self.deactivate(camera)

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await self.deactivate(camera)
        for process in self._processes:
            try:
                process.kill()
            except Exception:
                self.log.info('could not kill process')

    def request_backup(self) -> None:
        self.needs_backup = True


def process_image(data: bytes, rotation: ImageRotation, crop: Rectangle = None) -> bytes:
    image = PIL.Image.open(io.BytesIO(data))
    if crop is not None:
        image = image.crop((int(crop.x), int(crop.y), int(crop.x+crop.width), int(crop.y+crop.height)))
    image = image.rotate(int(rotation), expand=True, resample=PIL.Image.Resampling.BICUBIC)
    img_byte_arr = io.BytesIO()
    image.save(img_byte_arr, format='JPEG')
    return img_byte_arr.getvalue()
