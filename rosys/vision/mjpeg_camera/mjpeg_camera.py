import logging
from typing import Any

from ...geometry import Rectangle
from ..camera import ConfigurableCamera, TransformableCamera
from ..image_rotation import ImageRotation
from .mjpeg_capture import MjpegCapture


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
        self.device: MjpegCapture | None = None
        super().__init__(id=id, name=name, connect_after_init=connect_after_init,
                         base_path_overwrite=base_path_overwrite, **kwargs)
        self.log = logging.getLogger(f'rosys.vision.mjpeg_camera.{self.id}')
        self.username = username
        self.password = password

        parts = self.id.split('-')
        self.index: int | None = int(parts[1]) if len(parts) == 2 and parts[1].isdigit() else None
        self.mac = parts[0]
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

    @property
    def rotation(self) -> ImageRotation:
        return self._rotation

    @rotation.setter
    def rotation(self, rotation: ImageRotation) -> None:
        self._rotation = rotation
        if self.device is not None:
            self.device.rotation = rotation

    @property
    def crop(self) -> Rectangle | None:
        return self._crop

    @crop.setter
    def crop(self, crop: Rectangle | None) -> None:
        self._crop = crop
        if self.device is not None:
            self.device.crop = crop

    async def connect(self) -> None:
        async with self._device_connection():
            if self.device is not None:
                if self.device.is_active:
                    return
                await self._tear_down_device()
            self.device = MjpegCapture(camera_id=self.id, mac=self.mac, ip=self.ip, index=self.index,
                                       username=self.username, password=self.password,
                                       rotation=self.rotation, crop=self.crop, parameters=self.parameters,
                                       reconnect_interval=self.reconnect_interval,
                                       on_image=self._add_image,
                                       on_connect=self._apply_all_parameters)

    async def disconnect(self) -> None:
        async with self._device_connection():
            await self._tear_down_device()

    async def _tear_down_device(self) -> None:
        """Tear down the device. The caller must hold `device_connection_lock`."""
        if self.device is None:
            return
        await self.device.shutdown()
        self.device = None

    async def _set_fps(self, fps: int) -> None:
        assert self.device is not None
        await self.device.call('set_fps', fps)

    async def _get_fps(self) -> int | None:
        assert self.device is not None
        return await self.device.call('get_fps')

    async def _set_resolution(self, resolution: tuple[int, int]) -> None:
        assert self.device is not None
        await self.device.call('set_resolution', *resolution)

    async def _get_resolution(self) -> tuple[int, int] | None:
        assert self.device is not None
        return await self.device.call('get_resolution')

    async def _set_mirrored(self, mirrored: bool) -> None:
        assert self.device is not None
        await self.device.call('set_mirrored', mirrored)

    async def _get_mirrored(self) -> bool | None:
        assert self.device is not None
        return await self.device.call('get_mirrored')
