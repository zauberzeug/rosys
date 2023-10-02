import io
import logging
import re
import shutil
from typing import Any, Optional

import cv2
import numpy as np
import PIL
from cv2 import UMat

from .. import persistence, rosys
from ..geometry import Rectangle
from .camera_provider import CameraProvider
from .image import Image, ImageSize
from .usb_camera import ImageRotation, UsbCamera

SCAN_INTERVAL = 10


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


class UsbCameraProviderHardware(CameraProvider):
    """This module collects and provides real USB cameras.

    Camera devices are discovered through video4linux (v4l) and accessed with openCV.
    Therefore the program v4l2ctl and openCV (including python bindings) must be available.
    """

    def __init__(self) -> None:
        super().__init__()

        self.log = logging.getLogger('rosys.usb_camera_provider')

        self.last_scan: Optional[float] = None
        self._cameras: dict[str, UsbCamera] = {}

        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.capture_images, 0.2)
        rosys.on_repeat(self.update_parameters, 1)
        rosys.on_repeat(self.update_device_list, 1)

        self.needs_backup: bool = False
        # persistence.register(self)

    @property
    def cameras(self) -> dict[str, UsbCamera]:
        return self._cameras

    def backup(self) -> dict:
        return {'cameras': persistence.to_dict(self._cameras)}

    def restore(self, data: dict[str, Any]) -> None:
        persistence.replace_dict(self._cameras, UsbCamera, data.get('cameras', {}))

    async def capture_images(self) -> None:
        for uid, camera in self._cameras.items():
            try:
                if not camera.is_connected:
                    continue

                image = await rosys.run.io_bound(camera.capture_image)
                if image is None:
                    raise Exception(f'could not capture image from {uid}')

                device = camera.device
                if 'MJPG' in device.video_formats:
                    bytes_ = await rosys.run.io_bound(to_bytes, image)
                    if camera.crop or camera.rotation != ImageRotation.NONE:
                        bytes_ = await rosys.run.cpu_bound(process_jpeg_image, bytes_, camera.rotation, camera.crop)
                else:
                    bytes_ = await rosys.run.cpu_bound(process_ndarray_image, image, camera.rotation, camera.crop)
                if camera.crop:
                    size = ImageSize(width=int(camera.crop.width), height=int(camera.crop.height))
                elif camera.resolution:
                    size = camera.resolution
                else:
                    size = ImageSize(width=800, height=600)
                self.add_image(camera, Image(camera_id=uid, data=bytes_, time=rosys.time(), size=size))
            except Exception:
                self.log.exception(f'could not capture image from {uid}')
                await camera.deactivate()

    async def update_parameters(self) -> None:
        for camera in self._cameras.values():
            if camera.is_connected:
                await rosys.run.io_bound(camera.set_parameters)

    async def update_device_list(self) -> None:
        if self.last_scan is not None and rosys.time() < self.last_scan + SCAN_INTERVAL:
            return
        self.last_scan = rosys.time()
        output = await rosys.run.sh(['v4l2-ctl', '--list-devices'])
        if output is None:
            self.log.error('Could not scan for USB cameras. Is video4linux (lib4vl) installed?')
            return
        output = '\n'.join([s for s in output.split('\n') if not s.startswith('Cannot open device')])
        for infos in output.split('\n\n'):
            match = re.search(r'\((.*)\)', infos)
            if match is None:
                continue
            uid = match.group(1)
            if uid not in self._cameras:
                self.add_camera(UsbCamera(id=uid))
            lines = infos.splitlines()
            if 'dev/video' not in lines[1]:
                continue
            if not self._cameras[uid].is_connected:
                await self._cameras[uid].activate()

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.deactivate()

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('v4l2-ctl') is not None
