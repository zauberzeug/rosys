from __future__ import annotations

import abc
import asyncio
from collections import deque
from contextlib import asynccontextmanager
from typing import AsyncGenerator, Optional

from ... import rosys
from ...event import Event
from ..image import Image
from ..image_route import create_image_route


class Camera(abc.ABC):
    MAX_IMAGES = 256

    def __init__(self,
                 *,
                 id: str,  # pylint: disable=redefined-builtin
                 name: Optional[str] = None,
                 connect_after_init: bool = True,
                 streaming: bool = True,
                 image_grab_interval: float = 0.1,
                 base_path_overwrite: Optional[str] = None,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.id: str = id
        self.name = name or self.id
        self.connect_after_init = connect_after_init
        self.images: deque[Image] = deque(maxlen=self.MAX_IMAGES)
        self.streaming: bool = streaming
        self.base_path: str = f'images/{base_path_overwrite or id}'

        self.NEW_IMAGE: Event = Event()

        self.device_connection_lock: asyncio.Condition = asyncio.Condition()

        create_image_route(self)

        if connect_after_init:
            if asyncio.get_event_loop().is_running():
                asyncio.create_task(self.connect())
            else:
                rosys.on_startup(self.connect)

        async def stream() -> None:
            if self.streaming:
                await self.capture_image()
        rosys.on_repeat(stream, interval=image_grab_interval)

    def get_image_url(self, image: Image) -> str:
        return f'{self.base_path}/{image.time}'

    def get_latest_image_url(self) -> str:
        image = self.latest_captured_image
        if image is None:
            return f'{self.base_path}/placeholder'
        return self.get_image_url(image)

    def to_dict(self) -> dict:
        return {
            'id': self.id,
            'name': self.name,
            'connect_after_init': self.connect_after_init,
            'streaming': self.streaming,
        }

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

    async def connect(self) -> None:
        pass

    async def disconnect(self) -> None:
        pass

    async def reconnect(self) -> None:
        await self.disconnect()
        await self.connect()

    @property
    def captured_images(self) -> list[Image]:
        return [i for i in self.images if i.data]

    @property
    def latest_captured_image(self) -> Optional[Image]:
        return next((i for i in reversed(self.captured_images) if i.data), None)

    @property
    def latest_detected_image(self) -> Optional[Image]:
        return next((i for i in reversed(self.captured_images) if i.detections), None)

    def get_recent_images(self, *, current_time: Optional[float] = None, timespan: float = 10.0) -> list[Image]:
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
        raise NotImplementedError('Implement capture_image() in your camera class!')
