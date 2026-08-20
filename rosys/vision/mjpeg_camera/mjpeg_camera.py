import asyncio
import logging
from typing import Any

from ..camera import ConfigurableCamera, TransformableCamera
from .mjpeg_capture import MjpegCapture
from .vendors import VendorType, mac_to_vendor


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

        self._capture: MjpegCapture | None = None

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
        return self._capture is not None and self._capture.is_running

    async def connect(self) -> None:
        if self.is_connected:
            return

        if not self.ip:
            self.log.error('No IP address provided')
            return

        if mac_to_vendor(self.mac) == VendorType.OTHER:
            self.log.error('Could not connect to device: unknown vendor for mac="%s"', self.mac)
            return

        self._capture = MjpegCapture(camera_id=self.id, mac=self.mac, ip=self.ip, index=self.index,
                                     username=self.username, password=self.password,
                                     rotation=self.rotation, crop=self.crop, parameters=self.parameters,
                                     loop=asyncio.get_running_loop(), on_image=self._add_image)
        self._capture.start()

    async def disconnect(self) -> None:
        if self._capture is None:
            return
        await self._capture.shutdown()
        self._capture = None

    async def _set_fps(self, fps: int) -> None:
        if self._capture is not None:
            await self._capture.call('set_fps', fps)

    async def _get_fps(self) -> int | None:
        return await self._capture.call('get_fps') if self._capture is not None else None

    async def _set_resolution(self, resolution: tuple[int, int]) -> None:
        if self._capture is not None:
            await self._capture.call('set_resolution', *resolution)

    async def _get_resolution(self) -> tuple[int, int] | None:
        return await self._capture.call('get_resolution') if self._capture is not None else None

    async def _set_mirrored(self, mirrored: bool) -> None:
        if self._capture is not None:
            await self._capture.call('set_mirrored', mirrored)

    async def _get_mirrored(self) -> bool | None:
        return await self._capture.call('get_mirrored') if self._capture is not None else None
