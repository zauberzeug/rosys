from enum import Enum


class VendorType(Enum):
    JOVISION = 1
    DAHUA = 2
    AXIS = 3
    OTHER = -1


def mac_to_vendor(mac: str) -> VendorType:
    if mac.startswith('e0:62:90'):
        return VendorType.JOVISION
    if mac.startswith(('e4:24:6c', '3c:e3:6b', 'd4:43:0e', 'fc:5f:49')):
        return VendorType.DAHUA
    return VendorType.OTHER


def mac_to_url(mac: str, ip: str, substream: int = 0) -> str | None:
    vendor = mac_to_vendor(mac)
    if vendor == VendorType.JOVISION:
        return f'rtsp://admin:admin@{ip}/profile{substream}'
    if vendor == VendorType.DAHUA:
        return f'rtsp://admin:Adminadmin@{ip}/cam/realmonitor?channel=1&subtype={substream}'
    return None
