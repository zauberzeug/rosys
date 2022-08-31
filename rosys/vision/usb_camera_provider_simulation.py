import io
import random
import time
from typing import Any, Optional

import numpy as np
import PIL as pil
import rosys
from rosys import persistence

from .camera_provider import CameraProvider
from .image import Image, ImageSize
from .usb_camera import UsbCamera


class UsbCameraProviderSimulation(CameraProvider):
    '''This module collects and simulates USB cameras and generates synthetic images.

    In the current implementation the images only contain the camera ID and the current time.
    '''

    def __init__(self) -> None:
        super().__init__()

        self._cameras: dict[str, UsbCamera] = {}

        rosys.on_repeat(self.step, 1.0)
        rosys.on_repeat(lambda: self.prune_images(max_age_seconds=1.0), 5.0)

        self.needs_backup: bool = False
        persistence.register(self)

    @property
    def cameras(self) -> dict[str, UsbCamera]:
        return self._cameras

    def backup(self) -> dict:
        return {'cameras': persistence.to_dict(self._cameras)}

    def restore(self, data: dict[str, Any]) -> None:
        persistence.replace_dict(self._cameras, UsbCamera, data.get('cameras', {}))

    async def step(self) -> None:
        for camera in self._cameras.values():
            if not camera.active:
                continue
            assert camera.image_resolution is not None, 'simulated USB cameras should have an image resolution'
            image = Image(time=rosys.time(), camera_id=camera.id, size=camera.image_resolution)
            if rosys.is_test:
                image.data = b'test data'
            else:
                image.data = await rosys.run.cpu_bound(self.create_image_data, camera)
            camera.images.append(image)

    @staticmethod
    def create(uid: str, width: int = 800, height: int = 600, color: Optional[str] = None) -> UsbCamera:
        color = color or f'#{random.randint(0, 0xffffff):06x}'
        return UsbCamera(id=uid, resolution=ImageSize(width=width, height=height), color=color)

    @staticmethod
    def create_calibrated(uid: str, *,
                          width: int = 800, height: int = 600, color: Optional[str] = None,
                          x: float = 0.0, y: float = 0.0, z: float = 1.0,
                          roll: float = np.pi, pitch: float = 0.0, yaw: float = 0.0) -> UsbCamera:
        camera = UsbCameraProviderSimulation.create(uid, width, height, color)
        camera.set_perfect_calibration(width=width, height=height, x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)
        return camera

    @staticmethod
    def create_image_data(camera: UsbCamera) -> bytes:
        size = camera.image_resolution
        img = pil.Image.new('RGB', size=(size.width, size.height), color=camera.color)
        d = pil.ImageDraw.Draw(img)
        text = f'{camera.id}: {time.time()}'
        d.text((img.width / 2 - len(text) * 3 + 0, img.height / 2 - 5), text, fill=(0, 0, 0))
        d.text((img.width / 2 - len(text) * 3 + 1, img.height / 2 - 4), text, fill=(255, 255, 255))
        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        return img_byte_arr.getvalue()
