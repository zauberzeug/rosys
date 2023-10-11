import asyncio
import io
import logging
import shlex
import subprocess
from dataclasses import dataclass
from io import BytesIO
from typing import Any, AsyncGenerator, Optional

import imgsize
import PIL

import rosys

from .. import persistence
from ..geometry import Rectangle
from .camera import Camera, TransformCameraMixin
from .image import ImageSize
from .image_rotation import ImageRotation


class PeekableBytesIO(io.BytesIO):

    def peek(self, n=-1):
        position = self.tell()
        data = self.read(n)
        self.seek(position)
        return data


def process_image(data: bytes, rotation: ImageRotation, crop: Optional[Rectangle] = None) -> bytes:
    image = PIL.Image.open(io.BytesIO(data))
    if crop is not None:
        image = image.crop((int(crop.x), int(crop.y), int(crop.x + crop.width), int(crop.y + crop.height)))
    image = image.rotate(int(rotation), expand=True)  # NOTE: PIL handles rotation with 90 degree steps efficiently
    img_byte_arr = io.BytesIO()
    image.save(img_byte_arr, format='JPEG')
    return img_byte_arr.getvalue()


@dataclass(slots=True, kw_only=True)
class RtspCamera(TransformCameraMixin, Camera):
    capture_task: Optional[asyncio.Task] = None
    capture_process: Optional[subprocess.Popen] = None
    goal_fps: int = 6

    detect: bool = False
    url: Optional[str] = None
    authorized: bool = True

    @property
    def is_connected(self) -> bool:
        return self.capture_task is not None

    async def capture_images(self) -> None:
        async def stream() -> AsyncGenerator[bytes, None]:
            assert self.url is not None
            # if platform.system() == 'Darwin':
            #     command = f'gst-launch-1.0 rtspsrc location="{self.url}" latency=0 protocols=tcp drop-on-latency=true buffer-mode=none ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate=6/1" ! jpegenc ! fdsink'
            # else:
            logging.info(f'capture images from {self.id} with URL {self.url}')
            assert self.url is not None
            if 'subtype=0' in self.url:
                # to try: replace avdec_h264 with nvh264dec ! nvvidconv (!videoconvert)
                url = self.url.replace('subtype=0', 'subtype=1')
                command = f'gst-launch-1.0 rtspsrc location="{url}" latency=0 protocols=tcp ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate={self.goal_fps}/1"! jpegenc ! fdsink'
            else:
                command = f'gst-launch-1.0 rtspsrc location="{self.url}" latency=0 protocols=tcp ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate={self.goal_fps}/1" ! jpegenc ! fdsink'
            process = await asyncio.create_subprocess_exec(*shlex.split(command), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            assert process.stdout is not None
            assert process.stderr is not None
            self.capture_process = process

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
            logging.info(f'process {process.pid} exited with {process.returncode} and error {error.decode()}')
            if 'Unauthorized' in error.decode():
                self.authorized = False

        async for image in stream():
            if self.crop or self.rotation != ImageRotation.NONE:
                image = await rosys.run.cpu_bound(process_image, image, self.rotation, self.crop)
            try:
                with PeekableBytesIO(image) as f:
                    width, height = imgsize.get_size(f)
            except imgsize.UnknownSize:
                continue
            size = ImageSize(width=width, height=height)
            self.resolution = size
        self.capture_task = None

    async def activate(self) -> None:
        if self.is_connected:
            return
        task = rosys.background_tasks.create(self.capture_images(), name=f'capture {self.id}')
        if task is None:
            logging.warning(f'could not create task for {self.id}')
            return
        self.capture_task = task

    async def deactivate(self) -> None:
        if self.capture_task is not None:
            self.capture_task.cancel()
            self.capture_task = None
        if self.capture_process is not None:
            try:
                self.capture_process.kill()
            except Exception:
                logging.info('could not kill process')
