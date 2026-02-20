"""Generic MJPEG device for vendors without vendor-specific implementations."""

from collections.abc import Awaitable, Callable

from .mjpeg_device import MjpegDevice
from .vendors import VendorType, mac_to_vendor


class GenericMjpegDevice(MjpegDevice):
    """MJPEG device for generic vendors (e.g. Goodcam) using standard MJPEG streaming."""

    def __init__(self, mac: str, ip: str, *,
                 username: str | None = None,
                 password: str | None = None,
                 on_new_image_data: Callable[[bytes, float], Awaitable | None]) -> None:
        super().__init__(mac, ip, username=username, password=password, on_new_image_data=on_new_image_data)

        vendor = mac_to_vendor(mac)
        if vendor != VendorType.GOODCAM:
            raise ValueError(f'GenericMjpegDevice can only be used with GOODCAM devices. Got {vendor} for mac="{mac}"')
