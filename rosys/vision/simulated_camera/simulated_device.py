import random
from collections.abc import Awaitable, Callable

import numpy as np
import PIL.Image
import PIL.ImageDraw

from ... import rosys
from ...geometry import Point
from ..image import Image, ImageSize


class SimulatedDevice:

    def __init__(self,
                 id: str,  # pylint: disable=redefined-builtin
                 *,
                 size: ImageSize,
                 on_new_image: Callable[[Image], Awaitable | None],
                 color: str = '#ffffff',
                 fps: float = 30.0,
                 reconnect_interval: float = 3.0,
                 simulate_failing: bool = False) -> None:
        self._id = id
        self._size = size
        self.color = color
        self._on_new_image = on_new_image
        self.reconnect_interval = reconnect_interval
        self.simulate_failing = simulate_failing
        self._connected: bool = True
        self._last_connect_time: float = rosys.time()
        self._disconnect_time: float | None = None

        self._repeater = rosys.on_repeat(self._step, interval=1.0 / fps)

    @property
    def is_connected(self) -> bool:
        return self._connected

    def disconnect(self) -> None:
        """Simulate a connection loss (e.g. a bad cable); the device reconnects itself after `reconnect_interval`."""
        if not self._connected:
            return
        self._connected = False
        self._disconnect_time = rosys.time()

    async def _step(self) -> None:
        if not self._connected:
            assert self._disconnect_time is not None
            if rosys.time() - self._disconnect_time >= self.reconnect_interval:
                self._connected = True
                self._last_connect_time = rosys.time()
                self._disconnect_time = None
            return
        if self.simulate_failing and \
                random.random() < (rosys.time() - self._last_connect_time) / 30.0 * self._repeater.interval:
            self.disconnect()
            return
        await self._create_image()

    async def _create_image(self) -> None:
        timestamp = rosys.time()
        image: Image
        if rosys.is_test:
            image = _create_simple_image(self._id, self._size, self.color, timestamp)
        else:
            image = _create_image_data(self._id, self._size, self.color, timestamp)
        result = self._on_new_image(image)
        if isinstance(result, Awaitable):
            await result

    def set_fps(self, fps: float) -> None:
        self._repeater.interval = 1.0 / fps

    def get_fps(self) -> float:
        return 1.0 / self._repeater.interval


def _create_image_data(id: str, size: ImageSize, color: str, timestamp: float) -> Image:  # pylint: disable=redefined-builtin
    img = PIL.Image.new('RGB', size=(size.width, size.height), color=color)
    d = PIL.ImageDraw.Draw(img)
    text = f'{id}: {timestamp:.2f}'
    position = _floating_text_position(size.width, size.height, timestamp)

    d.text((position.x, position.y), text, fill=(0, 0, 0))
    d.text((position.x + 1, position.y + 1), text, fill=(255, 255, 255))

    return Image.from_pil(img, time=timestamp, camera_id=id)


def _create_simple_image(camera_id: str, size: ImageSize, color: str, timestamp) -> Image:
    img = PIL.Image.new('RGB', size=(size.width, size.height), color=color)
    return Image.from_pil(img, time=timestamp, camera_id=camera_id)


def _floating_text_position(box_width: int, box_height: int, timestamp: float, speed: float = 100, angle: float = np.deg2rad(45)) -> Point:
    """Calculate the position of the text that floats around the image."""
    return Point(x=(speed * timestamp * np.cos(angle)) % box_width,
                 y=(speed * timestamp * np.sin(angle)) % box_height)
