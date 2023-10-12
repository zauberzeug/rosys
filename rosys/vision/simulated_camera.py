import io
import math
import time
from dataclasses import dataclass
from typing import Optional

import PIL as pil

import rosys

from .camera import CalibratedCameraMixin, Camera
from .image import Image, ImageSize


@dataclass(slots=True, kw_only=True)
class SimulatedCameraDevice:
    video_id: str
    size: ImageSize
    creation_time: float = time.time()
    color: str = '#ffffff'

    @staticmethod
    def floating_text_position(t: float, box_width: int, box_height: int) -> tuple[float, float]:
        speed = 100
        angle_degrees = 45

        # Convert angle to radians
        angle_radians = math.radians(angle_degrees)

        # Calculate position at the given time
        x = (speed * t * math.cos(angle_radians)) % (box_width)
        y = (speed * t * math.sin(angle_radians)) % (box_height)

        return x, y

    def create_image_data(self) -> bytes:

        img = pil.Image.new('RGB', size=(self.size.width, self.size.height), color=self.color)
        d = pil.ImageDraw.Draw(img)
        text = f'{self.video_id}: {time.time():.2f}'
        text_pos_x, text_pos_y = SimulatedCameraDevice.floating_text_position(
            time.time(), self.size.width, self.size.height)

        d.text((text_pos_x, text_pos_y), text, fill=(0, 0, 0))
        d.text((text_pos_x + 1, text_pos_y + 1), text, fill=(255, 255, 255))

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        return img_byte_arr.getvalue()


@dataclass(slots=True, kw_only=True)
class SimulatedCamera(CalibratedCameraMixin, Camera):
    device: Optional[SimulatedCameraDevice] = None
    resolution: ImageSize
    _color: str

    @property
    def is_connected(self) -> bool:
        return self.device is not None

    async def connect(self) -> None:
        if self.device is None:
            self.device = SimulatedCameraDevice(video_id=self.id, color=self._color, size=self.resolution)

    async def disconnect(self) -> None:
        self.device = None

    async def capture_image(self) -> Optional[Image]:
        if self.device is None:
            return None
        image = Image(time=rosys.time(), camera_id=self.id, size=self.resolution)
        if rosys.is_test:
            image.data = b'test data'
        else:
            image.data = await rosys.run.cpu_bound(self.device.create_image_data)
        return image
