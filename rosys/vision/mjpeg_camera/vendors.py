from enum import Enum


class VendorType(Enum):
    AXIS = 1
    MOTEC = 2
    OTHER = -1


def mac_to_vendor(mac: str) -> VendorType:
    if mac.startswith(('00:40:8c', 'ac:cc:8e', 'b8:a4:4f', 'e8:27:25')):
        return VendorType.AXIS
    if mac.startswith('2c:26:5f'):
        return VendorType.MOTEC
    return VendorType.OTHER


def mac_to_url(mac: str, ip: str, *, index: int | None = None) -> str | None:
    vendor = mac_to_vendor(mac)
    if vendor == VendorType.AXIS:
        return f'http://{ip}/axis-cgi/mjpg/video.cgi?' + (f'camera={index}' if index is not None else '')
    if vendor == VendorType.MOTEC:
        return f'http://{ip}:1001/stream.mjpg'
    return None
