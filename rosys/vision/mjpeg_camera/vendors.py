from typing import Optional


class VendorType:
    AXIS = 1
    OTHER = -1


def mac_to_vendor(mac: str) -> int:
    if mac.startswith('00:40:8c') or mac.startswith('ac:cc:8e')\
            or mac.startswith('b8:a4:4f') or mac.startswith('e8:27:25'):
        return VendorType.AXIS
    return VendorType.OTHER


def mac_to_url(mac: str, ip: str) -> Optional[str]:
    vendor = mac_to_vendor(mac)
    if vendor == VendorType.AXIS:
        return f'http://{ip}/axis-cgi/mjpg/video.cgi'
    return None
