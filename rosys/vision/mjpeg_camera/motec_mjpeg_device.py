from collections.abc import Awaitable, Callable

from .mjpeg_device import MjpegDevice
from .motec_settings_interface import MotecSettingsInterface
from .vendors import VendorType, mac_to_vendor


class MotecMjpegDevice(MjpegDevice):
    def __init__(self, mac: str, ip: str | None = None, *,
                 username: str | None = '',
                 password: str | None = '',
                 control_port: int | None = 8885,
                 on_new_image_data: Callable[[bytes, float], Awaitable | None],
                 on_connect: Callable[[], Awaitable | None] | None = None,
                 reconnect_interval: float = 3.0) -> None:
        vendor = mac_to_vendor(mac)
        if vendor != VendorType.MOTEC:
            raise ValueError(f'MotecMjpegDevice can only be used with MOTEC devices. Got {vendor} for mac="{mac}"')

        self._control_port = control_port or 8885

        super().__init__(mac, ip, username=username, password=password,
                         on_new_image_data=on_new_image_data, on_connect=on_connect,
                         reconnect_interval=reconnect_interval)

    @property
    def settings_interface(self) -> MotecSettingsInterface:
        assert self.ip is not None, 'cannot reach the camera settings without an address'
        return MotecSettingsInterface(self.ip, port=self._control_port)

    async def set_fps(self, fps: int) -> None:
        await self.settings_interface.set_fps(fps)

    async def get_fps(self) -> int:
        return await self.settings_interface.get_fps()

    async def set_resolution(self, width: int, height: int) -> None:
        await self.settings_interface.set_stream_resolution(width, height)

    async def get_resolution(self) -> tuple[int, int]:
        return await self.settings_interface.get_stream_resolution()

    async def set_stream_port(self, port: int) -> None:
        await self.settings_interface.set_stream_port(port)

    async def get_stream_port(self) -> int:
        return await self.settings_interface.get_stream_port()

    async def get_stream_compression(self) -> int:
        return await self.settings_interface.get_stream_compression()

    async def set_stream_compression(self, level: int) -> None:
        await self.settings_interface.set_stream_compression(level)
