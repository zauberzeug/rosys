from typing import Optional

from .mjpeg_device import MjpegDevice
from .motec_settings_interface import MotecSettingsInterface
from .vendors import VendorType, mac_to_vendor


class MotecMjpegDevice(MjpegDevice):
    def __init__(self, mac: str, ip: str, *,
                 username: Optional[str] = '',
                 password: Optional[str] = '',
                 control_port: Optional[int] = 8885) -> None:

        super().__init__(mac, ip,
                         username=username, password=password)

        vendor = mac_to_vendor(mac)
        if not vendor == VendorType.MOTEC:
            raise ValueError(f'MotecMjpegDevice can only be used with MOTEC devices. Got {vendor} for mac="{mac}"')

        self.settings_interface = MotecSettingsInterface(ip, port=control_port or 8885)

    async def set_fps(self, fps: int) -> None:
        await self.settings_interface.set_fps(fps)

    async def set_resolution(self, width: int, height: int) -> None:
        await self.settings_interface.set_stream_resolution(width, height)

    async def set_stream_port(self, port: int) -> None:
        await self.settings_interface.set_stream_port(port)

    async def set_stream_compression(self, level: int) -> None:
        await self.settings_interface.set_stream_compression(level)
