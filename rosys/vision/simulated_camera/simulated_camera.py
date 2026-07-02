import random

from ..camera.configurable_camera import ConfigurableCamera
from ..camera.transformable_camera import TransformableCamera
from ..image import ImageSize
from .simulated_device import SimulatedDevice


class SimulatedCamera(ConfigurableCamera, TransformableCamera):

    def __init__(self,
                 *,
                 id: str,  # pylint: disable=redefined-builtin
                 name: str | None = None,
                 connect_after_init: bool = True,
                 width: int = 800,
                 height: int = 600,
                 color: str | None = None,
                 fps: int = 5,
                 reconnect_interval: float = 3.0,
                 simulate_failing: bool = False,
                 **kwargs,
                 ) -> None:
        super().__init__(id=id,
                         name=name,
                         connect_after_init=connect_after_init,
                         **kwargs)
        self.device: SimulatedDevice | None = None
        self.resolution = ImageSize(width=width, height=height)
        self.reconnect_interval = reconnect_interval
        self.simulate_failing = simulate_failing
        self._register_parameter('color', self._get_color, self._set_color,
                                 color or f'#{random.randint(0, 0xffffff):06x}')
        self._register_parameter('fps', self._get_fps, self._set_fps,
                                 min_value=1, max_value=30, step=1, default_value=fps)

    def to_dict(self) -> dict:
        return super().to_dict() | {
            'width': self.resolution.width,
            'height': self.resolution.height,
            'reconnect_interval': self.reconnect_interval,
        } | {
            name: param.value for name, param in self._parameters.items()
        }

    @property
    def is_connected(self) -> bool:
        return self.device is not None and self.device.is_connected

    @property
    def is_active(self) -> bool:
        return self.device is not None and self.device.is_active

    async def connect(self) -> None:
        if self.is_active:
            return
        if self.device is not None:
            await self.disconnect()
        self.device = SimulatedDevice(id=self.id, size=self.resolution, fps=self.parameters['fps'],
                                      on_new_image=self._add_image,
                                      reconnect_interval=self.reconnect_interval,
                                      simulate_failing=self.simulate_failing)
        await self._apply_all_parameters()

    async def disconnect(self) -> None:
        if self.device is None:
            return
        self.device.shutdown()
        self.device = None

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
