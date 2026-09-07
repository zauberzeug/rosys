import random

from ...helpers.deprecation import deprecated_param
from ..camera.configurable_camera import ConfigurableCamera
from ..camera.transformable_camera import TransformableCamera
from ..image import ImageSize
from .simulated_device import SimulatedDevice


class SimulatedCamera(ConfigurableCamera, TransformableCamera):

    @deprecated_param('width')
    @deprecated_param('height', stacklevel=3)
    def __init__(self,
                 *,
                 id: str,  # pylint: disable=redefined-builtin
                 name: str | None = None,
                 connect_after_init: bool = True,
                 resolution: tuple[int, int] = (800, 600),
                 width: int | None = None,
                 height: int | None = None,
                 color: str | None = None,
                 fps: int = 5,
                 **kwargs,
                 ) -> None:
        super().__init__(id=id,
                         name=name,
                         connect_after_init=connect_after_init,
                         **kwargs)
        self.device: SimulatedDevice | None = None
        if width is not None or height is not None:
            resolution = (width or resolution[0], height or resolution[1])
        self._register_parameter('resolution', self._get_resolution, self._set_resolution, resolution)
        self._register_parameter('color', self._get_color, self._set_color,
                                 color or f'#{random.randint(0, 0xffffff):06x}')
        self._register_parameter('fps', self._get_fps, self._set_fps,
                                 min_value=1, max_value=30, step=1, default_value=fps)

    def to_dict(self) -> dict:
        return super().to_dict() | {
            name: param.value for name, param in self._parameters.items()
        }

    @classmethod
    def args_from_dict(cls, data: dict) -> dict:
        data = super().args_from_dict(data)
        if 'width' in data and 'height' in data:
            data['resolution'] = (data.pop('width'), data.pop('height'))
        return data

    @property
    def is_connected(self) -> bool:
        return self.device is not None

    async def connect(self) -> None:
        if not self.is_connected:
            width, height = self.parameters['resolution']
            self.device = SimulatedDevice(id=self.id, size=ImageSize(width=width, height=height),
                                          fps=self.parameters['fps'], on_new_image=self._add_image)
            await self._apply_all_parameters()

    async def disconnect(self) -> None:
        self.device = None

    def _set_resolution(self, value: tuple[int, int]) -> None:
        assert self.device is not None
        self.device.size = ImageSize(width=value[0], height=value[1])

    def _get_resolution(self) -> tuple[int, int]:
        assert self.device is not None
        return (self.device.size.width, self.device.size.height)

    def _set_color(self, value: str) -> None:
        assert self.device is not None
        self.device.color = value

    def _get_color(self) -> str | None:
        assert self.device is not None
        return self.device.color

    def _set_fps(self, value: int) -> None:
        assert self.device is not None
        self.device.set_fps(value)

    def _get_fps(self) -> int:
        assert self.device is not None
        return int(self.device.get_fps())
