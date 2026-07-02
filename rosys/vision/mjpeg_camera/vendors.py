from enum import Enum


class VendorType(Enum):
    AXIS = 1
    MOTEC = 2
    GOODCAM = 3
    OPENIPC_ZAUBERZEUG = 4
    ARKVISION = 5
    OTHER = -1


def mac_to_vendor(mac: str) -> VendorType:
    if mac.startswith(('00:40:8c', 'ac:cc:8e', 'b8:a4:4f', 'e8:27:25')):
        return VendorType.AXIS
    if mac.startswith('2c:26:5f'):
        return VendorType.MOTEC
    if mac.startswith('2c:6f:51'):
        return VendorType.GOODCAM
    if mac.startswith('7a:7a:21'):
        return VendorType.OPENIPC_ZAUBERZEUG
    if mac.startswith('18:fd:cb'):  # NOTE: prefix observed on a single ArkCam Basic+ mini; may need expanding
        return VendorType.ARKVISION
    return VendorType.OTHER


def mac_to_url(mac: str, ip: str, *, index: int | None = None) -> str | None:
    vendor = mac_to_vendor(mac)
    if vendor == VendorType.AXIS:
        return f'http://{ip}/axis-cgi/mjpg/video.cgi?' + (f'camera={index}' if index is not None else '')
    if vendor == VendorType.MOTEC:
        return f'http://{ip}:1001/stream.mjpg'
    if vendor == VendorType.GOODCAM:
        return f'http://{ip}/api/v1/streams/secondary/stream.mjpeg'
    if vendor == VendorType.OPENIPC_ZAUBERZEUG:
        return f'http://{ip}/mjpeg'
    if vendor == VendorType.ARKVISION:
        return f'http://{ip}:81/'
    return None
