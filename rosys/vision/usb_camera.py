import io
import re
from dataclasses import dataclass, field
from typing import Any, Optional

import cv2
import numpy as np
import PIL
from cv2 import UMat

import rosys

from .. import persistence
from .camera import Camera, ConfigurableCameraMixin, TransformCameraMixin
from .image import Image, ImageSize
from .image_processing import process_jpeg_image, process_ndarray_image, to_bytes
from .image_rotation import ImageRotation

MJPG = cv2.VideoWriter_fourcc(*'MJPG')


async def get_video_devices_info() -> list[str]:
    output = await rosys.run.sh(['v4l2-ctl', '--list-devices'])
    if output is None:
        return None
    output = '\n'.join([s for s in output.split('\n') if not s.startswith('Cannot open device')])
    return output.split('\n\n')


async def find_video_id(camera_uid: str) -> Optional[int]:
    device_infos = await get_video_devices_info()
    if device_infos is None:
        return None

    for infos in device_infos:
        match = re.search(r'\((.*)\)', infos)
        if match is None:
            continue
        uid = match.group(1)
        if not uid == camera_uid:
            continue

        lines = infos.splitlines()
        if 'dev/video' not in lines[1]:
            continue

        num = int(lines[1].strip().lstrip('/dev/video'))

        return num


class UsbCameraHardwareDevice:
    video_id: int
    capture: cv2.VideoCapture
    exposure_min: int = 0
    exposure_max: int = 0
    exposure_default: int = 0
    has_manual_exposure: bool = False
    last_state: dict = dict()
    video_formats: set[str] = set()

    def __init__(self, video_id: int, capture: cv2.VideoCapture):
        self.video_id = video_id
        self.capture = capture
        self.set_video_format()

    @staticmethod
    async def from_uid(camera_id: str) -> Optional['UsbCameraHardwareDevice']:
        video_id = await find_video_id(camera_id)
        if video_id is None:
            return None

        capture = UsbCameraHardwareDevice.create_capture(video_id)

        if capture is None:
            return None

        return UsbCameraHardwareDevice(video_id=video_id, capture=capture)

    async def load_value_ranges(self) -> None:
        output = await self.run_v4l('--all')
        if output is None:
            return
        match = re.search(r'exposure_absolute.*: min=(\d*).*max=(\d*).*default=(\d*).*', output)
        if match is not None:
            self.has_manual_exposure = True
            self.exposure_min = int(match.group(1))
            self.exposure_max = int(match.group(2))
            self.exposure_default = int(match.group(3))
        else:
            self.has_manual_exposure = False
        output = await self.run_v4l('--list-formats')
        matches = re.finditer(r"$.*'(.*)'.*", output)
        for m in matches:
            self.video_formats.add(m.group(1))

    async def run_v4l(self, *args) -> str:
        cmd = ['v4l2-ctl', '-d', str(self.video_id)]
        cmd.extend(args)
        return await rosys.run.sh(cmd)

    def set_video_format(self) -> None:
        if 'MJPG' in self.video_formats:
            # NOTE enforcing motion jpeg for now
            if self.capture.get(cv2.CAP_PROP_FOURCC) != MJPG:
                self.capture.set(cv2.CAP_PROP_FOURCC, MJPG)
            # NOTE disable video decoding (see https://stackoverflow.com/questions/62664621/read-jpeg-frame-from-mjpeg-self-without-decoding-in-python-opencv/70869738?noredirect=1#comment110818859_62664621)
            if self.capture.get(cv2.CAP_PROP_CONVERT_RGB) != 0:
                self.capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
            # NOTE make sure there is no lag (see https://stackoverflow.com/a/30032945/364388)
            if self.capture.get(cv2.CAP_PROP_BUFFERSIZE) != 1:
                self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    @staticmethod
    def create_capture(index: int):
        capture = cv2.VideoCapture(index)
        if capture is None:
            return None
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            return None
        return capture


class UsbCamera(ConfigurableCameraMixin, TransformCameraMixin, Camera):
    device: Optional[UsbCameraHardwareDevice]
    detect: bool
    color: Optional[str]

    def __init__(self, id, name=None, connect_after_init=True, streaming=True) -> None:
        ConfigurableCameraMixin.__init__(self)
        TransformCameraMixin.__init__(self)
        Camera.__init__(self, id=id, name=name, connect_after_init=connect_after_init, streaming=streaming)
        self.device = None
        self.detect = False
        self.color = None

        self._register_parameter(name='auto_exposure', setter=self.set_exposure,
                                 getter=self.get_exposure, default_value=True)
        self._register_parameter(name='exposure', setter=self.set_exposure, getter=self.get_exposure, default_value=0)
        self._register_parameter(name='width', setter=self.set_width, getter=self.get_width, default_value=800)
        self._register_parameter(name='height', setter=self.set_height, getter=self.get_height, default_value=600)
        self._register_parameter(name='fps', setter=self.set_fps, getter=self.get_fps, default_value=10)

    @property
    def is_connected(self) -> bool:
        return self.device is not None

    async def connect(self) -> None:
        if self.is_connected:
            return
        device = await UsbCameraHardwareDevice.from_uid(self.id)
        if device is None:
            # logger.error(f'could not activate {self.id}')
            return

        self.device = device

        # logger.info(f'activated {self.id}')

    async def disconnect(self) -> None:
        if not self.is_connected:
            return

        assert self.device is not None
        await rosys.run.io_bound(self.device.capture.release)
        self.device = None

        # logger.info(f'deactivated {self.id}')

    async def capture_image(self) -> None:
        if not self.is_connected:
            return None
        assert self.device.capture is not None

        _, image = self.device.capture.read()

        device = self.device

        if 'MJPG' in device.video_formats:
            bytes_ = await rosys.run.io_bound(to_bytes, image)
            if self.crop or self.rotation != ImageRotation.NONE:
                bytes_ = await rosys.run.cpu_bound(process_jpeg_image, bytes_, self.rotation, self.crop)
        else:
            bytes_ = await rosys.run.cpu_bound(process_ndarray_image, image, self.rotation, self.crop)

        image = Image(time=rosys.time(), camera_id=self.id, size=self.image_resolution, data=bytes_)
        self._add_image(image)

    async def set_auto_exposure(self, auto: bool) -> None:
        if not self.is_connected:
            return

        device = self.device

        if device.has_manual_exposure:
            is_auto_exposure = await self.get_auto_exposure()
            if auto and not is_auto_exposure:
                # self.log.info(f'activating auto-exposure for {self.id}')
                device.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            else:
                device.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
                await self.set_exposure(self._parameters['exposure'].value)

    async def set_exposure(self, value: int) -> None:
        if not self.is_connected:
            return

        device = self.device
        assert device.capture is not None

        if device.has_manual_exposure:
            is_auto_exposure = await self.get_auto_exposure()
            if not is_auto_exposure:
                exposure = device.capture.get(cv2.CAP_PROP_EXPOSURE) / device.exposure_max
                if value != exposure:
                    device.capture.set(cv2.CAP_PROP_EXPOSURE, int(value * device.exposure_max))

    async def get_auto_exposure(self) -> Optional[bool]:
        if not self.is_connected:
            return None
        device = self.device
        return device.capture.get(cv2.CAP_PROP_AUTO_EXPOSURE) == 3

    async def get_exposure(self) -> Optional[int]:
        if not self.is_connected:
            return None
        device = self.device
        if not device.has_manual_exposure:
            return None
        return device.capture.get(cv2.CAP_PROP_EXPOSURE) / device.exposure_max

    async def set_width(self, width: int) -> None:
        if not self.is_connected:
            return
        device = self.device
        device.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)

    async def get_width(self) -> int:
        if not self.is_connected:
            return None
        device = self.device
        return int(device.capture.get(cv2.CAP_PROP_FRAME_WIDTH))

    async def set_height(self, height: int) -> None:
        if not self.is_connected:
            return
        device = self.device
        device.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    async def get_height(self) -> int:
        if not self.is_connected:
            return None
        device = self.device
        return int(device.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    async def set_fps(self, fps: int) -> None:
        if not self.is_connected:
            return
        device = self.device
        device.capture.set(cv2.CAP_PROP_FPS, fps)

    async def get_fps(self) -> int:
        if not self.is_connected:
            return None
        device = self.device
        return int(device.capture.get(cv2.CAP_PROP_FPS))
