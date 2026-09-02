import logging
import random
from collections.abc import Awaitable, Callable

import numpy as np
import PIL.Image
import PIL.ImageDraw

from ... import rosys
from ...geometry import Point
from ..capture_device import CaptureDevice, CaptureState
from ..image import Image, ImageSize

MEAN_TIME_TO_FAILURE = 30.0
"""Average seconds a simulated stream survives while `simulate_failing` is set."""


class SimulatedDevice(CaptureDevice):

    def __init__(self,
                 id: str,  # pylint: disable=redefined-builtin
                 *,
                 size: ImageSize,
                 on_new_image: Callable[[Image], Awaitable | None],
                 on_connect: Callable[[], Awaitable | None] | None = None,
                 color: str = '#ffffff',
                 fps: float = 30.0,
                 reconnect_interval: float = 3.0,
                 simulate_failing: bool = False) -> None:
        super().__init__(name=id,
                         log=logging.getLogger('rosys.vision.simulated_camera.simulated_device.' + id),
                         reconnect_interval=reconnect_interval)
        self._id = id
        self._size = size
        self.color = color
        self._on_new_image = on_new_image
        self._on_connect = on_connect
        self.simulate_failing = simulate_failing
        self._frame_interval = 1.0 / fps

        self._start_capture_task()
        self._state = CaptureState.STREAMING  # a simulated camera is reachable the moment it exists

    def disconnect(self) -> None:
        """Simulate a connection loss (e.g. a bad cable); the device reconnects itself after `reconnect_interval`."""
        if self._state is CaptureState.STREAMING:
            self._state = CaptureState.CONNECTING

    async def _run_session(self) -> None:
        self._set_state(CaptureState.STREAMING)
        await self._invoke_on_connect()
        session_start = rosys.time()
        while self._keeps_running() and self.is_connected:
            await rosys.sleep(self._frame_interval)
            if not self.is_connected:
                break
            if self.simulate_failing and \
                    random.random() < (rosys.time() - session_start) / MEAN_TIME_TO_FAILURE * self._frame_interval:
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

    async def _invoke_on_connect(self) -> None:
        """Notify the owner that the device has reconnected, e.g. to reapply camera parameters."""
        if self._on_connect is None:
            return
        result = self._on_connect()
        if isinstance(result, Awaitable):
            await result

    def set_fps(self, fps: float) -> None:
        self._frame_interval = 1.0 / fps

    def get_fps(self) -> float:
        return 1.0 / self._frame_interval


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
