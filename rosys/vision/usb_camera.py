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
from ..geometry import Rectangle
from .camera import Camera, ExposureCameraMixin, TransformCameraMixin
from .image import Image, ImageSize
from .image_rotation import ImageRotation

MJPG = cv2.VideoWriter_fourcc(*'MJPG')


def process_jpeg_image(data: bytes, rotation: ImageRotation, crop: Optional[Rectangle] = None) -> bytes:
    image = PIL.Image.open(io.BytesIO(data))
    if crop is not None:
        image = image.crop((int(crop.x), int(crop.y), int(crop.x+crop.width), int(crop.y+crop.height)))
    if rotation == ImageRotation.LEFT:
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif rotation == ImageRotation.RIGHT:
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    elif rotation == ImageRotation.UPSIDE_DOWN:
        image = cv2.rotate(image, cv2.ROTATE_180)
    img_byte_arr = io.BytesIO()
    image.save(img_byte_arr, format='JPEG')
    return img_byte_arr.getvalue()


def process_ndarray_image(image: np.ndarray, rotation: ImageRotation, crop: Optional[Rectangle] = None) -> bytes:
    if crop is not None:
        image = image[int(crop.y):int(crop.y+crop.height), int(crop.x):int(crop.x+crop.width)]
    if rotation == ImageRotation.LEFT:
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif rotation == ImageRotation.RIGHT:
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    elif rotation == ImageRotation.UPSIDE_DOWN:
        image = cv2.rotate(image, cv2.ROTATE_180)
    return cv2.imencode('.jpg', image)[1].tobytes()


def to_bytes(image: Any) -> bytes:
    return image[0].tobytes()


@dataclass(slots=True, kw_only=True)
class UsbCameraHardwareDevice:
    video_id: int
    capture: cv2.VideoCapture
    exposure_min: int = 0
    exposure_max: int = 0
    exposure_default: int = 0
    has_manual_exposure: bool = False
    last_state: dict = field(default_factory=dict)
    video_formats: set[str] = field(default_factory=set)

    # TODO should this be a dataclass?

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


class UsbCameraHardwareDeviceFactory:
    @staticmethod
    async def device_from_uid(camera_id: str) -> Optional[UsbCameraHardwareDevice]:
        video_id = await UsbCameraHardwareDeviceFactory.find_video_id(camera_id)
        if video_id is None:
            return None

        capture = UsbCameraHardwareDeviceFactory.create_capture(video_id)

        if capture is None:
            return None

        return UsbCameraHardwareDevice(video_id=video_id, capture=capture)

    @staticmethod
    async def get_video_devices_info() -> list[str]:
        output = await rosys.run.sh(['v4l2-ctl', '--list-devices'])
        if output is None:
            return None
        output = '\n'.join([s for s in output.split('\n') if not s.startswith('Cannot open device')])
        return output.split('\n\n')

    @staticmethod
    async def find_video_id(camera_uid: str) -> Optional[int]:
        device_infos = await UsbCameraHardwareDeviceFactory.get_video_devices_info()
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

    @staticmethod
    def create_capture(index: int):
        capture = cv2.VideoCapture(index)
        if capture is None:
            return None
        if not capture.isOpened():
            capture.release()
            return None
        return capture


@dataclass(slots=True, kw_only=True)
class UsbCamera(ExposureCameraMixin, TransformCameraMixin, Camera):
    device: Optional[UsbCameraHardwareDevice] = field(default=None, metadata=persistence.exclude)
    detect: bool = False
    color: Optional[str] = None

    @property
    def is_connected(self) -> bool:
        return self.device is not None

    async def activate(self) -> None:
        if self.is_connected:
            return
        device = await UsbCameraHardwareDeviceFactory.device_from_uid(self.id)
        if device is None:
            # logger.error(f'could not activate {self.id}')
            return

        self.device = device
        # logger.info(f'activated {self.id}')

    async def deactivate(self) -> None:
        if not self.is_connected:
            return

        assert self.device is not None
        await rosys.run.io_bound(self.device.capture.release)
        self.device = None

        # logger.info(f'deactivated {self.id}')

    async def capture_image(self) -> cv2.UMat:
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

        return Image(time=rosys.time(), camera_id=self.id, size=self.image_resolution, data=bytes_)

    async def set_exposure(self) -> None:
        if not self.is_connected:
            return

        device = self.device
        assert device.capture is not None
        if not self.auto_exposure and self.exposure is None and device.exposure_max > 0:
            self.exposure = device.exposure_default / device.exposure_max

        if device.has_manual_exposure:
            auto_exposure = device.capture.get(cv2.CAP_PROP_AUTO_EXPOSURE) == 3
            if self.auto_exposure and not auto_exposure:
                # self.log.info(f'activating auto-exposure for {self.id}')
                device.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # `v4l2-ctl -L` says "3: Aperture Priority Mode"
            if auto_exposure and not self.auto_exposure:
                # self.log.info(f'deactivating auto-exposure of {self.id}')
                device.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # `v4l2-ctl -L` says "1: Manual Mode"
            if not self.auto_exposure and device.exposure_max > 0:
                exposure = device.capture.get(cv2.CAP_PROP_EXPOSURE) / device.exposure_max
                if self.exposure is not None and self.exposure != exposure:
                    # self.log.info(f'updating exposure of {self.id} from {exposure} to {self.exposure})')
                    device.capture.set(cv2.CAP_PROP_EXPOSURE, int(self.exposure * device.exposure_max))

    async def set_parameters(self) -> None:
        if not self.is_connected:
            return
        device = self.device

        await self.set_exposure()
        self.device.set_video_format()

        self.fps = int(device.capture.get(cv2.CAP_PROP_FPS))

        resolution = ImageSize(
            width=int(device.capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            height=int(device.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        )

        if self.resolution and self.resolution != resolution:
            # self.log.info(f'updating resolution of {self.id} from {resolution} to {self.resolution}')
            device.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution.width)
            device.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution.height)
