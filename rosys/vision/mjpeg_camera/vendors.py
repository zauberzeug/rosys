from enum import IntEnum
from typing import Optional


class VendorType(IntEnum):
    AXIS = 1
    OTHER = -1


def mac_to_vendor(mac: str) -> VendorType:
    if any(mac.startswith(prefix) for prefix in ['00:40:8c', 'ac:cc:8e', 'b8:a4:4f', 'e8:27:25']):
        return VendorType.AXIS
    return VendorType.OTHER


def mac_to_url(mac: str, ip: str, *, index: Optional[int] = None) -> Optional[str]:
    vendor = mac_to_vendor(mac)
    if vendor == VendorType.AXIS:
        return f'http://{ip}/axis-cgi/mjpg/video.cgi' + (f'?camera={index}' if index is not None else '')
    return None
