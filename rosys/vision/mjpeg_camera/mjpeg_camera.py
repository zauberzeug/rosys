import logging
from typing import Any

from ... import rosys
from ..camera import ConfigurableCamera, TransformableCamera
from ..image import Image
from ..image_processing import process_jpeg_image
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

        parts = self.id.split('-')
        self.index: int | None = int(parts[1]) if len(parts) == 2 and parts[1].isdigit() else None
        self.mac = parts[0]
        self.device: MjpegDevice | None = None
        self._ip: str | None = ip

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

    @property
    def is_active(self) -> bool:
        return (self.device is not None) and self.device.is_active

    @property
    def ip(self) -> str | None:
        """The address of the camera; a running device rebinds to a new one without being torn down."""
        return self._ip

    @ip.setter
    def ip(self, ip: str | None) -> None:
        self._ip = ip
        if self.device is not None:
            self.device.ip = ip

    async def connect(self) -> None:
        async with self._device_connection():
            if self.device is not None:
                if self.device.is_active:
                    return
                await self._tear_down_device()
            self.device = MjpegDeviceFactory.create(self.mac, self.ip, index=self.index, username=self.username,
                                                    password=self.password,
                                                    on_new_image_data=self._handle_new_image_data,
                                                    on_connect=self._apply_all_parameters,
                                                    reconnect_interval=self.reconnect_interval)

    async def disconnect(self) -> None:
        async with self._device_connection():
            await self._tear_down_device()

    async def _tear_down_device(self) -> None:
        """Tear down the device. The caller must hold `device_connection_lock`."""
        if self.device is None:
            return
        await self.device.shutdown()
        self.device = None

    async def _handle_new_image_data(self, image_bytes: bytes, timestamp: float) -> None:
        image: Image | None = None
        if self.crop or self.rotation != ImageRotation.NONE:
            image_array = await rosys.run.cpu_bound(process_jpeg_image, image_bytes, self.rotation, self.crop)
            if image_array is not None:
                image = Image.from_array(image_array, camera_id=self.id, time=timestamp)
        else:
            image = await rosys.run.cpu_bound(Image.from_jpeg_bytes, image_bytes, camera_id=self.id, time=timestamp)

        if image is not None:
            self._add_image(image)

    async def _set_fps(self, fps: int) -> None:
        assert self.device is not None
        await self.device.set_fps(fps)

    async def _get_fps(self) -> int | None:
        assert self.device is not None

        return await self.device.get_fps()

    async def _set_resolution(self, resolution: tuple[int, int]) -> None:
        assert self.device is not None

        await self.device.set_resolution(*resolution)

    async def _get_resolution(self) -> tuple[int, int] | None:
        assert self.device is not None

        return await self.device.get_resolution()

    async def _set_mirrored(self, mirrored: bool) -> None:
        assert self.device is not None

        await self.device.set_mirrored(mirrored)

    async def _get_mirrored(self) -> bool | None:
        assert self.device is not None

        return await self.device.get_mirrored()
