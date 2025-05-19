from enum import Enum


class VendorType(Enum):
    JOVISION = 1
    DAHUA = 2
    AXIS = 3
    REOLINK = 4
    UNIARCH = 5
    OTHER = -1


def mac_to_vendor(mac: str) -> VendorType:
    if mac.startswith('e0:62:90'):
        return VendorType.JOVISION
    if mac.startswith(('e4:24:6c', '3c:e3:6b', 'd4:43:0e', 'fc:5f:49', '30:dd:aa')):
        return VendorType.DAHUA
    if mac.startswith('ec:71:db'):
        return VendorType.REOLINK
    if mac.startswith('6c:f1:7e'):
        return VendorType.UNIARCH
    return VendorType.OTHER


def mac_to_url(mac: str, ip: str, substream: int = 0) -> str | None:
    vendor = mac_to_vendor(mac)
    if vendor == VendorType.JOVISION:
        return f'rtsp://admin:admin@{ip}/profile{substream}'
    if vendor == VendorType.DAHUA:
        return f'rtsp://admin:Adminadmin@{ip}/cam/realmonitor?channel=1&subtype={substream}'
    if vendor == VendorType.REOLINK:
        return f'rtsp://admin:Adminadmin@{ip}/Preview_01_{"sub" if substream else "main"}'
    if vendor == VendorType.UNIARCH:
        return f'rtsp://admin:Admin_adm1n@{ip}/media/video{2 if substream else 1}'
    return None
