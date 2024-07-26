import random

from typing_extensions import Self

from ... import rosys
from ..camera.configurable_camera import ConfigurableCamera
from ..camera.transformable_camera import TransformableCamera
from ..image import Image, ImageSize
from .simulated_device import SimulatedDevice


class SimulatedCamera(ConfigurableCamera, TransformableCamera):

    def __init__(self,
                 *,
                 id: str,  # pylint: disable=redefined-builtin
                 name: str | None = None,
                 connect_after_init: bool = True,
                 streaming: bool = True,
                 width: int = 800,
                 height: int = 600,
                 color: str | None = None,
                 fps: int = 5,
                 **kwargs,
                 ) -> None:
        super().__init__(id=id,
                         name=name,
                         connect_after_init=connect_after_init,
                         streaming=streaming,
                         polling_interval=1.0 / fps,
                         **kwargs)
        self.device: SimulatedDevice | None = None
        self.resolution = ImageSize(width=width, height=height)
        self._register_parameter('color', self._get_color, self._set_color,
                                 color or f'#{random.randint(0, 0xffffff):06x}')
        self._register_parameter('fps', self._get_fps, self._set_fps,
                                 min_value=1, max_value=30, step=1, default_value=fps)

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
            await self._apply_all_parameters()

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

    def _set_color(self, value: str) -> None:
        assert self.device is not None
        self.device.color = value

    def _get_color(self) -> str | None:
        assert self.device is not None
        return self.device.color

    def _set_fps(self, value: int) -> None:
        self.polling_interval = 1.0 / value

    def _get_fps(self) -> int:
        return int(1.0 / self.polling_interval)
