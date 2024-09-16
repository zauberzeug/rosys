import time
from collections.abc import Awaitable, Callable

import cv2
import numpy as np
import PIL

from ... import rosys
from ...geometry import Point
from ..image import ImageSize


class SimulatedDevice:

    def __init__(self,
                 id: str,  # pylint: disable=redefined-builtin
                 size: ImageSize,
                 image_data_callback: Callable[[bytes], Awaitable | None],
                 color: str = '#ffffff',
                 fps: float = 30.0) -> None:
        self.id = id
        self.size = size
        self.color = color
        self.image_data_callback = image_data_callback
        self.creation_time: float = rosys.time()

        self.repeater = rosys.on_repeat(self.capture_image, interval=1.0 / fps)

    @staticmethod
    def create_image_data(id: str, size: ImageSize, color: str) -> bytes:
        img = PIL.Image.new('RGB', size=(size.width, size.height), color=color)
        d = PIL.ImageDraw.Draw(img)
        text = f'{id}: {time.time():.2f}'
        position = floating_text_position(size.width, size.height)

        d.text((position.x, position.y), text, fill=(0, 0, 0))
        d.text((position.x + 1, position.y + 1), text, fill=(255, 255, 255))

        _, encoded_image = cv2.imencode('.jpg', np.array(img)[:, :, ::-1])  # NOTE: cv2 expects BGR
        return encoded_image.tobytes()

    async def capture_image(self) -> None:
        print(f'{rosys.time()}: capturing image')
        image_data = await rosys.run.cpu_bound(SimulatedDevice.create_image_data, self.id, self.size, self.color)
        if not image_data:
            return
        result = self.image_data_callback(image_data)
        if isinstance(result, Awaitable):
            await result

    def set_fps(self, fps: float) -> None:
        self.repeater.interval = 1.0 / fps

    def get_fps(self) -> float:
        return 1.0 / self.repeater.interval


def floating_text_position(box_width: int, box_height: int, speed: float = 100, angle: float = np.deg2rad(45)) -> Point:
    """Calculate the position of the text that floats around the image."""
    t = time.time()
    return Point(x=(speed * t * np.cos(angle)) % box_width,
                 y=(speed * t * np.sin(angle)) % box_height)
