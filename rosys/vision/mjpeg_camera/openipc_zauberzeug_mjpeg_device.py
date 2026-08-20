from ..openipc_zauberzeug_settings_interface import OpenIpcZauberzeugSettingsInterface
from .mjpeg_device import ImageDataHandler, MjpegDevice
from .vendors import VendorType, mac_to_vendor


class OpenIpcZauberzeugMjpegDevice(MjpegDevice):
    """MJPEG device for OpenIPC-Zauberzeug cameras running the divinus streamer.

    Extends :class:`MjpegDevice` with an :class:`OpenIpcZauberzeugSettingsInterface` so fps,
    resolution and mirroring are applied on the device via the divinus HTTP API
    instead of the no-op defaults of the base class.
    """

    def __init__(self, mac: str, ip: str, *,
                 username: str | None = None,
                 password: str | None = None,
                 on_new_image_data: ImageDataHandler) -> None:
        super().__init__(mac, ip, username=username, password=password, on_new_image_data=on_new_image_data)

        vendor = mac_to_vendor(mac)
        if vendor != VendorType.OPENIPC_ZAUBERZEUG:
            raise ValueError(f'OpenIpcZauberzeugMjpegDevice can only be used with '
                             f'OPENIPC_ZAUBERZEUG devices. Got {vendor} for mac="{mac}"')

        self.settings_interface = OpenIpcZauberzeugSettingsInterface(ip, username=username, password=password)

    async def set_fps(self, fps: int) -> None:
        await self.settings_interface.set_mjpeg_fps(fps)

    async def get_fps(self) -> int:
        return await self.settings_interface.get_mjpeg_fps() or 0

    async def set_resolution(self, width: int, height: int) -> None:
        await self.settings_interface.set_resolution(width, height)

    async def get_resolution(self) -> tuple[int, int]:
        return await self.settings_interface.get_resolution()

    async def set_mirrored(self, mirrored: bool) -> None:
        await self.settings_interface.set_mirrored(mirrored)

    async def get_mirrored(self) -> bool:
        return await self.settings_interface.get_mirrored() or False
