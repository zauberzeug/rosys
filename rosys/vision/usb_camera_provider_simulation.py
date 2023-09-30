import io
import random
import time
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np
import PIL as pil

from rosys import persistence

from .. import rosys
from .camera_provider import CameraProvider
from .image import Image, ImageSize
from .usb_camera import UsbCamera


@dataclass(slots=True, kw_only=True)
class UsbCameraSimulatedDevice:
    video_id: int
    creation_time: float = rosys.time()

    @staticmethod
    def floating_text_position(time: float, box_width: int, box_height: int) -> tuple[float, float]:
        speed = 100
        angle_degrees = 45

        # Convert angle to radians
        angle_radians = math.radians(angle_degrees)

        # Calculate position at the given time
        x = (speed * time * math.cos(angle_radians)) % (box_width)
        y = (speed * time * math.sin(angle_radians)) % (box_height)

        return x, y

    def create_image_data(self, camera: UsbCamera) -> bytes:
        size = camera.image_resolution
        assert size is not None
        img = pil.Image.new('RGB', size=(size.width, size.height), color=camera.color)
        d = pil.ImageDraw.Draw(img)
        text = f'{camera.id}: {time.time()}'
        text_pos_x, text_pos_y = UsbCameraSimulatedDevice.floating_text_position(time.time(), size.width, size.height)

        d.text((text_pos_x, text_pos_y), text, fill=(0, 0, 0))
        d.text((text_pos_x + 1, text_pos_y + 1), text, fill=(255, 255, 255))

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        return img_byte_arr.getvalue()

class UsbCameraProviderSimulation(CameraProvider):
    """This module collects and simulates USB cameras and generates synthetic images.

    In the current implementation the images only contain the camera ID and the current time.
    """
    USE_PERSISTENCE: bool = True

    def __init__(self) -> None:
        super().__init__()

        self._cameras: dict[str, UsbCamera] = {}

        rosys.on_repeat(self.step, 1.0)
        rosys.on_repeat(self.simulate_device_discovery, 5.0)

        self.needs_backup: bool = False
        if self.USE_PERSISTENCE:
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
            if not camera.is_connected:
                continue
            assert camera.image_resolution is not None, 'simulated USB cameras should have an image resolution'
            image = Image(time=rosys.time(), camera_id=camera.id, size=camera.image_resolution)
            if rosys.is_test:
                image.data = b'test data'
            else:
                image.data = await rosys.run.cpu_bound(camera.device.create_image_data, camera)
            self.add_image(camera, image)

    async def simulate_device_discovery(self) -> None:
        for camera in self._cameras.values():
            if not camera.is_connected:
                await self.activate(camera.id)
                continue
            # disconnect cameras rendomly with probabilty rising with time
            time_since_last_activation = rosys.time() - camera.device.creation_time
            if random.random() < time_since_last_activation / 30.:
                camera.device = None

    def get_next_video_id(self) -> int:
        return max([camera.device.video_id for camera in self._cameras.values() if camera.is_connected], default=0) + 1

    async def activate(self, camera_id: str) -> None:
        '''Simulates activating a camera by assigning it a mock video device.'''
        if camera_id not in self._cameras:
            raise ValueError(f'Camera with id {camera_id} is not managed by this provider')

        camera = self._cameras[camera_id]
        device = UsbCameraSimulatedDevice(video_id=self.get_next_video_id())
        camera.device = device

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
