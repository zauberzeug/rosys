import random
from typing import Optional, Self

from ... import rosys
from ..camera.configurable_camera import ConfigurableCamera
from ..image import Image, ImageSize
from .simulated_device import SimulatedDevice


class SimulatedCamera(ConfigurableCamera):

    def __init__(self,
                 *,
                 id: str,  # pylint: disable=redefined-builtin
                 name: Optional[str] = None,
                 connect_after_init: bool = True,
                 streaming: bool = True,
                 resolution=ImageSize(width=800, height=600),
                 color: Optional[str] = None,
                 **kwargs
                 ) -> None:
        super().__init__(id=id,
                         name=name,
                         connect_after_init=connect_after_init,
                         streaming=streaming,
                         **kwargs)
        self.device: Optional[SimulatedDevice] = None
        self.resolution = resolution
        if color is None:
            color = f'#{random.randint(0, 0xffffff):06x}'
        self._color = color

        self._register_parameter(name='color', setter=self.set_color, getter=self.get_color, default_value='#ffffff')

    def to_dict(self) -> dict:
        return {
            'id': self.id,
            'width': self.resolution.width,
            'height': self.resolution.height,
            'color': self._color,
        }

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(
            id=data['id'],
            resolution=ImageSize(width=data['width'], height=data['height']),
            color=data['color'],
        )

    @property
    def is_connected(self) -> bool:
        return self.device is not None

    async def connect(self) -> None:
        if not self.is_connected:
            self.device = SimulatedDevice(id=self.id, color=self._color, size=self.resolution)

    async def disconnect(self) -> None:
        self.device = None

    async def capture_image(self) -> None:
        if not self.is_connected:
            return None
        image = Image(time=rosys.time(), camera_id=self.id, size=self.resolution)
        if rosys.is_test:
            image.data = b'test data'
        else:
            assert self.device is not None
            image.data = await rosys.run.cpu_bound(self.device.create_image_data)
        self._add_image(image)

    async def set_color(self, val: str) -> None:
        if not self.is_connected:
            return
        assert self.device is not None

        self.device.color = val

    async def get_color(self) -> Optional[str]:
        if not self.is_connected:
            return None
        assert self.device is not None

        return self.device.color
