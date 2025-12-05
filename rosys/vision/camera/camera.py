from __future__ import annotations

import abc
import asyncio
import logging
from collections import deque
from collections.abc import AsyncGenerator
from contextlib import asynccontextmanager
from typing import Any, ClassVar, Self

from nicegui import Event

from ... import rosys
from ..image import Image
from ..image_route import create_image_route

logger = logging.getLogger('rosys.vision.camera')


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
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.id: str = id
        self.name = name or self.id
        self.connect_after_init = connect_after_init
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
        }

    @classmethod
    def args_from_dict(cls, data: dict[str, Any]) -> dict:
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(**cls.args_from_dict(data))

    @property
    def is_connected(self) -> bool:
        """To be interpreted as "ready to capture images"."""
        return False

    @asynccontextmanager
    async def _device_connection(self) -> AsyncGenerator[None, None]:
        await self.device_connection_lock.acquire()
        try:
            yield
        finally:
            self.device_connection_lock.release()

    async def connect(self) -> None:  # noqa: B027
        pass

    async def disconnect(self) -> None:  # noqa: B027
        pass

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
