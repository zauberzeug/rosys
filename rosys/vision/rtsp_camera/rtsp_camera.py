import logging
from typing import Any

from typing_extensions import Self

from ... import rosys
from ..camera.configurable_camera import ConfigurableCamera
from ..camera.transformable_camera import TransformableCamera
from ..image import Image
from ..image_processing import get_image_size_from_bytes, process_jpeg_image
from .rtsp_device import RtspDevice


class RtspCamera(ConfigurableCamera, TransformableCamera):

    def __init__(self,
                 *,
                 id: str,  # pylint: disable=redefined-builtin
                 name: str | None = None,
                 connect_after_init: bool = True,
                 fps: int = 5,
                 jovision_profile: int = 1,
                 bitrate: int = 4096,
                 ip: str | None = None,
                 **kwargs,
                 ) -> None:
        super().__init__(id=id,
                         name=name,
                         connect_after_init=connect_after_init,
                         **kwargs)

        self.log = logging.getLogger(f'rosys.vision.rtsp_camera.{self.id}')

        self.device: RtspDevice | None = None
        self.ip: str | None = ip

        self._register_parameter('jovision_profile', self.get_jovision_profile, self.set_jovision_profile,
                                 min_value=0, max_value=1, step=1, default_value=jovision_profile)
        self._register_parameter('fps', self.get_fps, self.set_fps,
                                 min_value=1, max_value=30, step=1, default_value=fps)
        self._register_parameter('bitrate', self.get_bitrate, self.set_bitrate,
                                 min_value=32, max_value=8192, step=1, default_value=bitrate)

    def to_dict(self) -> dict[str, Any]:
        return super().to_dict() | {
            name: param.value for name, param in self._parameters.items()
        } | {
            'ip': self.ip,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(**data)

    @property
    def is_connected(self) -> bool:
        return self.device is not None and self.device.is_connected

    @property
    def url(self) -> str | None:
        if not self.is_connected:
            return None
        assert self.device is not None

        return self.device.url

    async def connect(self) -> None:
        if self.is_connected:
            return

        if not self.ip:
            self.log.error('no IP address provided for camera %s', self.id)
            return

        self.device = RtspDevice(mac=self.id, ip=self.ip,
                                 jovision_profile=self.parameters['jovision_profile'],
                                 fps=self.parameters['fps'],
                                 on_new_image_data=self._handle_new_image_data)

        await self._apply_all_parameters()

    async def disconnect(self) -> None:
        if not self.is_connected:
            return
        logging.info('camera %s: disconnect initialized...', self.id)

        assert self.device is not None
        await self.device.shutdown()
        self.device = None

    async def _handle_new_image_data(self, image_bytes: bytes, timestamp: float) -> None:
        if not image_bytes:
            return
        transformed_image_bytes = await rosys.run.cpu_bound(process_jpeg_image, image_bytes, self.rotation, self.crop)
        if transformed_image_bytes is None:
            return

        try:
            final_image_resolution = get_image_size_from_bytes(transformed_image_bytes)
        except ValueError:
            return

        image = Image(time=timestamp, camera_id=self.id, size=final_image_resolution, data=transformed_image_bytes)
        self._add_image(image)

    async def set_fps(self, fps: int) -> None:
        assert self.device is not None

        await self.device.set_fps(fps)

    async def get_fps(self) -> int | None:
        assert self.device is not None

        return await self.device.get_fps()

    def set_jovision_profile(self, profile: int) -> None:
        assert self.device is not None

        self.device.set_jovision_profile(profile)

    def get_jovision_profile(self) -> int | None:
        assert self.device is not None

        return self.device.get_jovision_profile()

    async def set_bitrate(self, bitrate: int) -> None:
        assert self.device is not None

        await self.device.set_bitrate(bitrate)

    async def get_bitrate(self) -> int | None:
        assert self.device is not None

        return await self.device.get_bitrate()

    async def _apply_parameters(self, new_values: dict[str, Any], force_set: bool = False) -> None:
        await super()._apply_parameters(new_values, force_set)
        if self.is_connected:
            assert self.device is not None
            await self.device.restart_gstreamer()
