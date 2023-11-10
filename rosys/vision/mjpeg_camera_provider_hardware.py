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

import httpx
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


class MJpegCameraProviderHardware(CameraProvider, persistence.PersistentModule):
    """This module collects and provides real motion jpeg streaming cameras."""

    def __init__(self, *, frame_rate: int = 6) -> None:
        super().__init__()

        self.frame_rate = frame_rate

        self.log = logging.getLogger('rosys.mjpeg_camera_provider')

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
        rosys.on_repeat(lambda: self.prune_images(max_age_seconds=10.0), 5.0)

    @property
    def cameras(self) -> dict[str, RtspCamera]:
        return self._cameras

    def backup(self) -> dict:
        return {'cameras': persistence.to_dict(self._cameras)}

    def restore(self, data: dict[str, Any]) -> None:
        persistence.replace_dict(self._cameras, RtspCamera, data.get('cameras', {}))

    async def capture_images(self, camera: RtspCamera) -> None:
        self.log.info(f'Capturing images from {camera.url}')

        async def stream() -> AsyncGenerator[bytes, None]:
            async with httpx.AsyncClient() as client:
                assert camera.url is not None
                async with client.stream('GET', camera.url) as response:
                    response: httpx.Response
                    if response.status_code != 200:
                        self.log.error(
                            f'could not connect to {camera.url}: {response.status_code} {response.reason_phrase}')
                        return
                    buffer = BytesIO()
                    header = None
                    pos = 0
                    async for chunk in response.aiter_bytes():
                        buffer.write(chunk)
                        while True:
                            if header is None:
                                header_pos = buffer.getvalue().find(b'\xff\xd8', pos)
                                if header_pos == -1:
                                    pos = max(0, buffer.tell() - 1)
                                    break
                                pos = header_pos + 2
                                header = header_pos
                            else:
                                footer_pos = buffer.getvalue().find(b'\xff\xd9', pos)
                                if footer_pos == -1:
                                    pos = max(0, buffer.tell() - 1)
                                    break
                                image = buffer.getvalue()[header:footer_pos + 2]
                                yield image  # Or process the image as needed
                                buffer = BytesIO(buffer.getvalue()[footer_pos + 2:])
                                pos = 0
                                header = None

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

    async def create_camera(self, name: str, url: str) -> RtspCamera:
        camera = RtspCamera(id=name, url=url)
        self.log.info(f'adding camera {url} with id {name}')
        self.add_camera(camera)
        await self.activate(camera)
        return camera

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await self.deactivate(camera)
        for process in self._processes:
            try:
                process.kill()
            except Exception:
                self.log.info('could not kill process')


def process_image(data: bytes, rotation: ImageRotation, crop: Optional[Rectangle] = None) -> bytes:
    image = PIL.Image.open(io.BytesIO(data))
    if crop is not None:
        image = image.crop((int(crop.x), int(crop.y), int(crop.x + crop.width), int(crop.y + crop.height)))
    image = image.rotate(int(rotation), expand=True)  # NOTE: PIL handles rotation with 90 degree steps efficiently
    img_byte_arr = io.BytesIO()
    image.save(img_byte_arr, format='JPEG')
    return img_byte_arr.getvalue()
