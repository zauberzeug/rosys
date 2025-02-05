import logging
from typing import Any

from ... import rosys
from ..camera import ConfigurableCamera, TransformableCamera
from ..image import Image
from ..image_processing import get_image_size_from_bytes, process_jpeg_image
from ..image_rotation import ImageRotation
from .mjpeg_device import MjpegDevice
from .mjpeg_device_factory import MjpegDeviceFactory


class MjpegCamera(TransformableCamera, ConfigurableCamera):

    def __init__(self,
                 *,
                 id: str,  # pylint: disable=redefined-builtin
                 name: str | None = None,
                 connect_after_init: bool = True,
                 base_path_overwrite: str | None = None,
                 username: str | None = None,
                 password: str | None = None,
                 ip: str | None = None,
                 fps: int = 10,
                 resolution: tuple[int, int] = (640, 480),
                 mirrored: bool = False,
                 **kwargs: Any,
                 ) -> None:
        super().__init__(id=id, name=name, connect_after_init=connect_after_init,
                         base_path_overwrite=base_path_overwrite, **kwargs)
        self.log = logging.getLogger(f'rosys.vision.mjpeg_camera.{self.id}')
        self.username = username
        self.password = password

        self.ip = ip

        self.index: int | None = None
        parts = self.id.split('-')
        if len(parts) == 2 and parts[1].isdigit():
            self.index = int(parts[1])

        self.mac = parts[0]
        self.device: MjpegDevice | None = None

        self._register_parameter('fps', self._get_fps, self._set_fps, default_value=fps)
        self._register_parameter('resolution', self._get_resolution, self._set_resolution, default_value=resolution)
        self._register_parameter('mirrored', self._get_mirrored, self._set_mirrored, default_value=mirrored)

    def to_dict(self) -> dict:
        return super().to_dict() | {
            name: param.value for name, param in self._parameters.items()
        } | {
            'username': self.username,
            'password': self.password,
            'ip': self.ip,
        }

    @property
    def is_connected(self) -> bool:
        return (self.device is not None) and self.device.is_connected

    async def connect(self) -> None:
        if self.is_connected:
            return

        if not self.ip:
            self.log.error('No IP address provided')
            return

        try:
            self.device = MjpegDeviceFactory.create(self.mac, self.ip, index=self.index, username=self.username,
                                                    password=self.password, on_new_image_data=self._handle_new_image_data)
        except ValueError as error:
            self.log.error('Could not connect to device: %s', error)
            return

        await self._apply_all_parameters()

    async def disconnect(self) -> None:
        if self.device is None:
            return

        self.device.shutdown()
        self.device = None

    async def _handle_new_image_data(self, image: bytes, timestamp: float) -> None:
        if self.crop or self.rotation != ImageRotation.NONE:
            image_ = await rosys.run.cpu_bound(process_jpeg_image, image, self.rotation, self.crop)
            if image_ is None:
                return
            image = image_
        try:
            final_image_resolution = get_image_size_from_bytes(image)
        except ValueError:
            return

        self._add_image(Image(camera_id=self.id, data=image, time=timestamp, size=final_image_resolution))

    async def _set_fps(self, fps: int) -> None:
        assert self.device is not None
        await self.device.set_fps(fps)

    async def _get_fps(self) -> int:
        assert self.device is not None

        return await self.device.get_fps()

    async def _set_resolution(self, resolution: tuple[int, int]) -> None:
        assert self.device is not None

        await self.device.set_resolution(*resolution)

    async def _get_resolution(self) -> tuple[int, int]:
        assert self.device is not None

        return await self.device.get_resolution()

    async def _set_mirrored(self, mirrored: bool) -> None:
        assert self.device is not None

        await self.device.set_mirrored(mirrored)

    async def _get_mirrored(self) -> bool:
        assert self.device is not None

        return await self.device.get_mirrored()
