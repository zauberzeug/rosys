import asyncio
import hashlib
import io
import logging
import re
import shlex
import shutil
import subprocess
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
        self.arpscan_cmd = 'sudo /usr/sbin/arp-scan'
        #self.arpscan_cmd = 'arp-scan'

        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.update_device_list, 1)
        rosys.on_repeat(lambda: self.prune_images(max_age_seconds=1.0), 5.0)

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
            command = f'ffmpeg -i {camera.url} -f image2pipe -vf fps=fps=6 -nostats -y -fflags nobuffer -flags low_delay -strict experimental -rtsp_transport tcp -qscale:v 2 -qscale 2 -movflags +faststart -threads 1 -c:v mjpeg pipe:1'
            self.log.info(command)
            args = shlex.split(command)
            process = await asyncio.create_subprocess_exec(*args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
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

        size = ImageSize(width=1920, height=1080)
        async for image in stream():
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
        for interface in net.interfaces():
            output = (await rosys.run.sh(f'{self.arpscan_cmd} -I {interface} --localnet', timeout=10))
            if output is None:
                return

            for camera in self._cameras.values():
                camera.active = False
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
                    #self.log.debug('ignoring mac {mac} because it seems not to be a camera')
                    continue
                url = f'rtsp://admin:admin@{ip}/profile0'
                if mac not in self._cameras:
                    camera = RtspCamera(id=mac, url=url)
                    self._cameras[mac] = camera
                    self.log.info(f'adding camera {url}')
                    await self.CAMERA_ADDED.call(camera)
                self._cameras[mac].active = True
                self._cameras[mac].url = url
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
            process.kill()
