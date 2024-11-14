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
                 *,
                 size: ImageSize,
                 on_new_image_data: Callable[[bytes, float], Awaitable | None],
                 color: str = '#ffffff',
                 fps: float = 30.0) -> None:
        self._id = id
        self._size = size
        self.color = color
        self._on_new_image_data = on_new_image_data
        self._creation_time: float = rosys.time()

        self._repeater = rosys.on_repeat(self._create_image, interval=1.0 / fps)

    @property
    def creation_time(self) -> float:
        return self._creation_time

    async def _create_image(self) -> None:
        timestamp = rosys.time()
        image_data: bytes | None
        if rosys.is_test:
            image_data = b'test data'
        else:
            image_data = await rosys.run.cpu_bound(_create_image_data, self._id, self._size, self.color, timestamp)
        if not image_data:
            return
        result = self._on_new_image_data(image_data, timestamp)
        if isinstance(result, Awaitable):
            await result

    def set_fps(self, fps: float) -> None:
        self._repeater.interval = 1.0 / fps

    def get_fps(self) -> float:
        return 1.0 / self._repeater.interval


def _create_image_data(id: str, size: ImageSize, color: str, timestamp: float) -> bytes:  # pylint: disable=redefined-builtin
    img = PIL.Image.new('RGB', size=(size.width, size.height), color=color)
    d = PIL.ImageDraw.Draw(img)
    text = f'{id}: {timestamp:.2f}'
    position = _floating_text_position(size.width, size.height, timestamp)

    d.text((position.x, position.y), text, fill=(0, 0, 0))
    d.text((position.x + 1, position.y + 1), text, fill=(255, 255, 255))

    _, encoded_image = cv2.imencode('.jpg', np.array(img)[:, :, ::-1])  # NOTE: cv2 expects BGR
    return encoded_image.tobytes()


def _floating_text_position(box_width: int, box_height: int, timestamp: float, speed: float = 100, angle: float = np.deg2rad(45)) -> Point:
    """Calculate the position of the text that floats around the image."""
    return Point(x=(speed * timestamp * np.cos(angle)) % box_width,
                 y=(speed * timestamp * np.sin(angle)) % box_height)
