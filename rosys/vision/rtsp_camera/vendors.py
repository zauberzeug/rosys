from typing import Optional


class VendorType:
    JOVISION = 1
    DAHUA = 2
    OTHER = 3


def mac_to_vendor(mac: str) -> int:
    if mac.startswith('e0:62:90'):
        return VendorType.JOVISION
    if mac.startswith('e4:24:6c') or mac.startswith('3c:e3:6b'):
        return VendorType.DAHUA
    return VendorType.OTHER


def mac_to_url(mac: str, ip: str, jovision_profile: int = 0) -> Optional[str]:
    vendor = mac_to_vendor(mac)
    if vendor == VendorType.JOVISION:
        return f'rtsp://admin:admin@{ip}/profile{jovision_profile}'
    if vendor == VendorType.DAHUA:
        return f'rtsp://admin:Adminadmin@{ip}/cam/realmonitor?channel=1&subtype=0'
    return None
