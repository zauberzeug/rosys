from collections.abc import Awaitable, Callable

from .arkvision_settings_interface import ArkVisionSettingsInterface
from .mjpeg_device import MjpegDevice
from .vendors import VendorType, mac_to_vendor


class ArkVisionMjpegDevice(MjpegDevice):
    """MJPEG device for Ark Vision ArkCam Basic+ cameras.

    Images come from the MJPEG-over-HTTP stream on port 81, which is enabled on demand via the
    REST API. Frame rate, resolution and mirroring are adjusted through the same REST API.
    """

    def __init__(self, mac: str, ip: str, *,
                 index: int | None = None,
                 username: str | None = None,
                 password: str | None = None,
                 on_new_image_data: Callable[[bytes, float], Awaitable | None]) -> None:
        vendor = mac_to_vendor(mac)
        if vendor != VendorType.ARKVISION:
            raise ValueError(
                f'ArkVisionMjpegDevice can only be used with ARKVISION devices. Got {vendor} for mac="{mac}"')
        self.settings_interface = ArkVisionSettingsInterface(ip)
        super().__init__(mac, ip, index=index, username=username, password=password,
                         on_new_image_data=on_new_image_data)

    async def _prepare_stream(self) -> None:
        await self.settings_interface.enable_http_mjpeg()

    async def get_fps(self) -> int | None:
        return await self.settings_interface.get_fps()

    async def set_fps(self, fps: int) -> None:
        await self.settings_interface.set_fps(fps)

    async def get_resolution(self) -> tuple[int, int] | None:
        return await self.settings_interface.get_resolution()

    async def set_resolution(self, width: int, height: int) -> None:
        await self.settings_interface.set_resolution(width, height)

    async def get_mirrored(self) -> bool | None:
        return await self.settings_interface.get_mirrored()

    async def set_mirrored(self, mirrored: bool) -> None:
        await self.settings_interface.set_mirrored(mirrored)
