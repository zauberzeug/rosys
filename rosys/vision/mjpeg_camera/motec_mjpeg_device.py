from collections.abc import Awaitable, Callable

from .mjpeg_device import MjpegDevice
from .motec_settings_interface import MotecSettingsInterface
from .vendors import VendorType, mac_to_vendor


class MotecMjpegDevice(MjpegDevice):
    def __init__(self, mac: str, ip: str, *,
                 username: str | None = '',
                 password: str | None = '',
                 control_port: int | None = 8885,
                 on_new_image_data: Callable[[bytes, float], Awaitable | None]) -> None:

        super().__init__(mac, ip, username=username, password=password, on_new_image_data=on_new_image_data)

        vendor = mac_to_vendor(mac)
        if vendor != VendorType.MOTEC:
            raise ValueError(f'MotecMjpegDevice can only be used with MOTEC devices. Got {vendor} for mac="{mac}"')

        self.settings_interface = MotecSettingsInterface(ip, port=control_port or 8885)

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
