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
                 width: int = 800,
                 height: int = 600,
                 color: Optional[str] = None,
                 **kwargs,
                 ) -> None:
        super().__init__(id=id,
                         name=name,
                         connect_after_init=connect_after_init,
                         streaming=streaming,
                         **kwargs)
        self.device: Optional[SimulatedDevice] = None
        self.resolution = ImageSize(width=width, height=height)
        self._register_parameter('color', self.get_color, self.set_color,
                                 color or f'#{random.randint(0, 0xffffff):06x}')

    def to_dict(self) -> dict:
        return {
            'id': self.id,
            'width': self.resolution.width,
            'height': self.resolution.height,
        } | {
            name: param.value for name, param in self._parameters.items()
        }

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(**data)

    @property
    def is_connected(self) -> bool:
        return self.device is not None

    async def connect(self) -> None:
        if not self.is_connected:
            self.device = SimulatedDevice(id=self.id, size=self.resolution)
            self._apply_all_parameters()

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

    def set_color(self, val: str) -> None:
        assert self.device is not None
        self.device.color = val

    def get_color(self) -> Optional[str]:
        assert self.device is not None
        return self.device.color
