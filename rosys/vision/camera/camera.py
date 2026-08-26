from __future__ import annotations

import abc
import asyncio
import logging
import random
from collections import deque
from collections.abc import AsyncGenerator
from contextlib import asynccontextmanager
from typing import Any, ClassVar, Self

from nicegui import Event

from ... import rosys
from ..image import Image
from ..image_route import create_image_route

logger = logging.getLogger('rosys.vision.camera')

MIN_RECONNECT_INTERVAL = 0.1
"""Shortest wait between two connection attempts.

An interval of zero reads as "retry at once", which on a single event loop means a capture loop
that never lets anything else run.
"""

MAX_RECONNECT_INTERVAL = 60.0
"""Longest wait between two connection attempts, i.e. the cap of the back-off."""

RECONNECT_JITTER = 0.2
"""Relative spread of the wait between two connection attempts."""


def retry_delay(reconnect_interval: float, failed_attempts: int = 0) -> float:
    """How long a device waits before its next connection attempt.

    The wait doubles from the second consecutive failure on, up to `MAX_RECONNECT_INTERVAL`, so an
    unreachable camera stops hammering, and is jittered so cameras that went down together do not
    retry in lock-step.

    :param reconnect_interval: the configured wait after a stream that was working ended
    :param failed_attempts: how many attempts in a row have failed to deliver a single frame
    """
    interval = max(reconnect_interval, MIN_RECONNECT_INTERVAL)
    interval *= 2 ** min(max(failed_attempts - 1, 0), 16)
    interval *= random.uniform(1 - RECONNECT_JITTER, 1 + RECONNECT_JITTER)
    return min(max(interval, MIN_RECONNECT_INTERVAL), MAX_RECONNECT_INTERVAL)


class Camera(abc.ABC):
    IMAGE_HISTORY_LENGTH: ClassVar[int] = 16
    '''How many images are stored in the queue for retrieval via `captured_images` or `get_recent_images`.

    Changing this value only affects cameras created afterwards!
    '''

    def __init__(self,
                 *,
                 id: str,  # pylint: disable=redefined-builtin
                 name: str | None = None,
                 connect_after_init: bool = True,
                 streaming: bool | None = None,
                 polling_interval: float | None = None,
                 base_path_overwrite: str | None = None,
                 reconnect_interval: float = 3.0,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.id: str = id
        self.name = name or self.id
        self.connect_after_init = connect_after_init
        self._reconnect_interval = MIN_RECONNECT_INTERVAL
        self.reconnect_interval = reconnect_interval
        self.images: deque[Image] = deque(maxlen=Camera.IMAGE_HISTORY_LENGTH)
        self.base_path: str = f'images/{base_path_overwrite or id}'

        if streaming is not None:
            logger.warning('The `streaming` parameter has been removed. All cameras now stream images by default.')
        if polling_interval is not None:
            logger.warning('The `polling_interval` parameter has been removed. All cameras should now use callbacks.')

        self.NEW_IMAGE = Event[Image]()

        self.device_connection_lock: asyncio.Condition = asyncio.Condition()

        create_image_route(self)

        if connect_after_init:
            rosys.on_startup(self.connect)
        rosys.on_shutdown(self.disconnect)

    @property
    def reconnect_interval(self) -> float:
        """Seconds the device waits before reopening a stream that ended; the wait grows while attempts fail."""
        return self._reconnect_interval

    @reconnect_interval.setter
    def reconnect_interval(self, interval: float) -> None:
        self._reconnect_interval = clamp_reconnect_interval(interval, logger)

    @property
    def streaming(self) -> bool:
        logger.warning('The `streaming` parameter has been removed. All cameras now stream images by default.')
        return True

    @streaming.setter
    def streaming(self, value: bool) -> None:  # pylint: disable=unused-argument
        logger.warning('The `streaming` parameter has been removed. All cameras now stream images by default.')

    @property
    def polling_interval(self) -> float:
        logger.warning('The `polling_interval` parameter has been removed. All cameras should now use callbacks.')
        return 0.0

    @polling_interval.setter
    def polling_interval(self, value: float) -> None:  # pylint: disable=unused-argument
        logger.warning('The `polling_interval` parameter has been removed. All cameras should now use callbacks.')

    def get_image_url(self, image: Image) -> str:
        return f'{self.base_path}/{image.time}'

    def get_latest_image_url(self) -> str:
        image = self.latest_captured_image
        if image is None or not self.is_connected:
            return f'{self.base_path}/placeholder'
        return self.get_image_url(image)

    def to_dict(self) -> dict:
        base_path_id = self.base_path.replace('images/', '', 1) if self.base_path.startswith('images') else None
        return {
            'id': self.id,
            'name': self.name,
            'connect_after_init': self.connect_after_init,
            'base_path_overwrite': base_path_id if base_path_id != self.id else None,
            'reconnect_interval': self.reconnect_interval,
        }

    @classmethod
    def args_from_dict(cls, data: dict[str, Any]) -> dict:
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(**cls.args_from_dict(data))

    @property
    def is_connected(self) -> bool:
        """Whether the camera is currently streaming or capturing images."""
        return False

    @property
    def is_active(self) -> bool:
        """Whether a connection is wanted, i.e. whether the camera keeps itself connected."""
        return self.is_connected

    @property
    def is_reconnecting(self) -> bool:
        """Whether the camera is active but not currently streaming (waiting to reconnect)."""
        return self.is_active and not self.is_connected

    @asynccontextmanager
    async def _device_connection(self) -> AsyncGenerator[None, None]:
        """Serialize device creation and tear-down."""
        await self.device_connection_lock.acquire()
        try:
            yield
        finally:
            self.device_connection_lock.release()

    async def connect(self) -> None:  # noqa: B027
        """Create the self-healing device; it keeps retrying until ``disconnect()`` tears it down."""

    async def disconnect(self) -> None:  # noqa: B027
        """Tear down the device, which also stops its reconnection attempts."""

    async def reconnect(self) -> None:
        await self.disconnect()
        await self.connect()

    @property
    def captured_images(self) -> list[Image]:
        return list(self.images)

    @property
    def latest_captured_image(self) -> Image | None:
        return next((i for i in reversed(self.captured_images)), None)

    @property
    def latest_detected_image(self) -> Image | None:
        return next((i for i in reversed(self.captured_images) if i.detections), None)

    def get_recent_images(self, *, current_time: float | None = None, timespan: float = 10.0) -> list[Image]:
        """Returns all images that were captured. Latest images are at the end of the list.

        :param current_time: the starting time for the search; defaults to the current time
        :param timespan: the timespan to search back in seconds
        """
        if current_time is None:
            current_time = rosys.time()
        return [i for i in self.captured_images if i.time > current_time - timespan]

    def _add_image(self, image: Image) -> None:
        self.images.append(image)
        self.NEW_IMAGE.emit(image)

    async def capture_image(self) -> None:
        raise DeprecationWarning('The `capture_image()` method has been removed. All cameras should now use callbacks.')


def clamp_reconnect_interval(interval: float, log: logging.Logger) -> float:
    """Hold `interval` to `MIN_RECONNECT_INTERVAL`, saying so when the requested value cannot be honored."""
    if interval >= MIN_RECONNECT_INTERVAL:
        return interval
    log.warning('a reconnect interval of %.2f s is too short; using %.2f s', interval, MIN_RECONNECT_INTERVAL)
    return MIN_RECONNECT_INTERVAL
