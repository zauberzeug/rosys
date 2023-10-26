from __future__ import annotations

import abc
from collections import deque
from typing import Optional
from uuid import uuid4

from ... import rosys
from ...event import Event
from ..image import Image
from ..image_route import create_image_route


class Camera(abc.ABC):
    MAX_IMAGES = 256

    id: str
    images: deque[Image]
    name: Optional[str]
    base_path: str

    connect_after_init: bool
    streaming: bool

    fps: Optional[int]
    """current frames per second (read only)"""

    NEW_IMAGE: Event
    """a new image is available (argument: image)"""

    def __init__(self, id: str, *, name: Optional[str] = None, connect_after_init: bool = True, streaming: bool = True) -> None:
        super().__init__()
        self.id = id
        self.images = deque(maxlen=self.MAX_IMAGES)
        self.connect_after_init = connect_after_init
        self.streaming = streaming
        self.fps = None
        self.base_path = f'images/{str(uuid4())}'
        self.NEW_IMAGE = Event()

        create_image_route(self)

        if name is None:
            self.name = self.id
        else:
            self.name = name

        if self.connect_after_init:
            # start a new task to activate the camera
            rosys.on_startup(self.connect)

        async def stream() -> None:
            if self.streaming and self.is_connected:
                await self.capture_image()

        rosys.on_repeat(stream, interval=.01)

    def get_image_url(self, image: Image) -> str:
        return f'{self.base_path}/{image.time}'

    def get_latest_image_url(self) -> str:
        image = self.latest_captured_image
        if image is None:
            return f'{self.base_path}/placeholder'
        return self.get_image_url(image)

    @property
    def is_connected(self) -> bool:
        """To be interpreted as "ready to capture images"."""
        return False

    async def connect(self) -> None:
        # NOTE: this method may be executed concurrently even after any check for is_connected
        #       Make sure it is idempotent!
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
        images = self.captured_images
        return images[-1] if images else None

    def get_recent_images(self, current_time: float, timespan: float = 10.0) -> list[Image]:
        return [i for i in self.captured_images if i.time > current_time - timespan]

    def _add_image(self, image: Image) -> None:
        self.images.append(image)
        self.NEW_IMAGE.emit(image)

    async def capture_image(self) -> None:
        raise NotImplementedError("Implement capture_image() in your camera class!")
