import logging
from collections.abc import Awaitable, Callable

from .arkvision_mjpeg_device import ArkVisionMjpegDevice
from .axis_mjpeg_device import AxisMjpegDevice
from .mjpeg_device import MjpegDevice
from .motec_mjpeg_device import MotecMjpegDevice
from .openipc_zauberzeug_mjpeg_device import OpenIpcZauberzeugMjpegDevice
from .vendors import VendorType, mac_to_vendor

log = logging.getLogger('rosys.vision.mjpeg_camera.mjpeg_device_factory')


class MjpegDeviceFactory:
    @staticmethod
    def create(mac: str, ip: str | None = None, *,
               index: int | None = None,
               username: str | None = None,
               password: str | None = None,
               control_port: int | None = None,
               on_new_image_data: Callable[[bytes, float], Awaitable | None],
               on_connect: Callable[[], Awaitable | None] | None = None) -> MjpegDevice:

        vendor = mac_to_vendor(mac)

        if vendor == VendorType.AXIS:
            return AxisMjpegDevice(mac, ip,
                                   index=index, username=username, password=password,
                                   on_new_image_data=on_new_image_data, on_connect=on_connect)

        if vendor == VendorType.MOTEC:
            return MotecMjpegDevice(mac, ip,
                                    username=username, password=password, control_port=control_port,
                                    on_new_image_data=on_new_image_data, on_connect=on_connect)

        if vendor == VendorType.ARKVISION:
            return ArkVisionMjpegDevice(mac, ip,
                                        index=index, username=username, password=password,
                                        on_new_image_data=on_new_image_data, on_connect=on_connect)

        if vendor == VendorType.OPENIPC_ZAUBERZEUG:
            return OpenIpcZauberzeugMjpegDevice(mac, ip,
                                                username=username, password=password,
                                                on_new_image_data=on_new_image_data, on_connect=on_connect)

        if vendor is VendorType.OTHER:
            log.warning('unknown vendor for mac="%s", no stream URL known for this camera', mac)

        return MjpegDevice(mac, ip,
                           username=username, password=password,
                           on_new_image_data=on_new_image_data, on_connect=on_connect)
