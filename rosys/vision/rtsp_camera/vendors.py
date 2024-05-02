from enum import Enum
from typing import Optional


class VendorType(Enum):
    JOVISION = 1
    DAHUA = 2
    AXIS = 3
    OTHER = -1


def mac_to_vendor(mac: str) -> VendorType:
    if mac.startswith('e0:62:90'):
        return VendorType.JOVISION
    if mac.startswith('e4:24:6c') or mac.startswith('3c:e3:6b'):
        return VendorType.DAHUA
    if any(mac.startswith(prefix) for prefix in ['00:40:8c', 'ac:cc:8e', 'b8:a4:4f', 'e8:27:25']):
        return VendorType.AXIS
    return VendorType.OTHER


def mac_to_url(mac: str, ip: str, jovision_profile: int = 0) -> Optional[str]:
    vendor = mac_to_vendor(mac)
    if vendor == VendorType.JOVISION:
        return f'rtsp://admin:admin@{ip}/profile{jovision_profile}'
    if vendor == VendorType.DAHUA:
        return f'rtsp://admin:Adminadmin@{ip}/cam/realmonitor?channel=1&subtype=0'
    return None
