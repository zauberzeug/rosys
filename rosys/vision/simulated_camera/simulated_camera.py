import random
from typing import Optional

import rosys

from ..camera import Camera, ConfigurableCameraMixin
from ..image import Image, ImageSize
from .simulated_device import SimulatedDevice


class SimulatedCamera(ConfigurableCameraMixin, Camera):

    def __init__(self,
                 id, *,
                 name: Optional[str] = None,
                 connect_after_init: bool = True,
                 streaming: bool = True,
                 resolution=ImageSize(width=800, height=600),
                 color: Optional[str] = None,
                 ) -> None:
        super().__init__(self, id=id, name=name, connect_after_init=connect_after_init, streaming=streaming)
        self.device: Optional[SimulatedDevice] = None
        self.id = id
        self.resolution = resolution
        if color is None:
            color = f'#{random.randint(0, 0xffffff):06x}'
        self._color = color

        self._register_parameter(name='color', setter=self.set_color, getter=self.get_color, default_value='#ffffff')

    def __to_dict__(self) -> dict:
        return {
            'id': self.id,
            'width': self.resolution.width,
            'height': self.resolution.height,
            'color': self._color,
        }

    @staticmethod
    def from_dict(data: dict) -> 'SimulatedCamera':
        return SimulatedCamera(id=data['id'], resolution=ImageSize(width=data['width'], height=data['height']), color=data['color'])

    @property
    def is_connected(self) -> bool:
        return self.device is not None

    async def connect(self) -> None:
        if self.device is None:
            self.device = SimulatedDevice(video_id=self.id, color=self._color, size=self.resolution)

    async def disconnect(self) -> None:
        self.device = None

    async def capture_image(self) -> None:
        if self.device is None:
            return None
        image = Image(time=rosys.time(), camera_id=self.id, size=self.resolution)
        if rosys.is_test:
            image.data = b'test data'
        else:
            image.data = await rosys.run.cpu_bound(self.device.create_image_data)
        self._add_image(image)

    async def set_color(self, val: str) -> None:
        self.device.color = val

    async def get_color(self) -> str:
        return self.device.color
