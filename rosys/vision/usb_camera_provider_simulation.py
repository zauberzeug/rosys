import io
import random
import time
from typing import Any, Optional

import PIL as pil
import rosys
from rosys import persistence

from .camera_provider import CameraProvider
from .image import Image, ImageSize
from .usb_camera import UsbCamera


class UsbCameraProviderSimulation(CameraProvider):

    def __init__(self) -> None:
        super().__init__()

        self._cameras: dict[str, UsbCamera] = {}

        rosys.on_repeat(self.step, 1.0)

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
    def create_calibrated(uid: str, width: int = 800, height: int = 600, color: Optional[str] = None,
                          x: float = 0, y: float = 0, z: float = 1,
                          yaw: float = 0, tilt_x: float = 0, tilt_y: float = 0) -> UsbCamera:
        camera = UsbCameraProviderSimulation.create(uid, width, height, color)
        camera.set_perfect_calibration(x, y, z, yaw, tilt_x, tilt_y, width, height)
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