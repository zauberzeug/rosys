from enum import Enum


class VendorType(Enum):
    JOVISION = 1
    DAHUA = 2
    AXIS = 3
    REOLINK = 4
    UNIARCH = 5
    ANJOY = 6
    OPENIPC = 7
    OTHER = -1


mac_prefix_to_vendor: dict[str, VendorType] = {
    'e0:62:90': VendorType.JOVISION,
    'e4:24:6c': VendorType.DAHUA,
    '3c:e3:6b': VendorType.DAHUA,
    'd4:43:0e': VendorType.DAHUA,
    'fc:5f:49': VendorType.DAHUA,
    '30:dd:aa': VendorType.DAHUA,
    'ec:71:db': VendorType.REOLINK,
    '6c:f1:7e': VendorType.UNIARCH,
    'f0:00:06': VendorType.ANJOY,
    '7a:7a:21': VendorType.OPENIPC,
}


def _vendor_to_url(vendor_type: VendorType, ip: str, substream: int) -> str | None:
    # pylint: disable=too-many-return-statements
    match vendor_type:
        case VendorType.JOVISION:
            return f'rtsp://admin:admin@{ip}/profile{substream}'
        case VendorType.DAHUA:
            return f'rtsp://admin:Adminadmin@{ip}/cam/realmonitor?channel=1&subtype={substream}'
        case VendorType.REOLINK:
            return f'rtsp://admin:Adminadmin@{ip}/Preview_01_{"sub" if substream else "main"}'
        case VendorType.UNIARCH:
            return f'rtsp://admin:Admin_adm1n@{ip}/media/video{2 if substream else 1}'
        case VendorType.ANJOY:
            return f'rtsp://admin:123456@{ip}/h264/ch{2 if substream else 1}'
        case VendorType.OPENIPC:
            return f'rtsp://root:Adminadmin@{ip}/stream={substream}'
        case VendorType.OTHER:
            return None
    raise AssertionError('unreachable')  # Just for mypy


def mac_to_vendor(mac: str) -> VendorType:
    prefix = mac[:8]
    return mac_prefix_to_vendor.get(prefix, VendorType.OTHER)


def mac_to_url(mac: str, ip: str, substream: int = 0) -> str | None:
    vendor = mac_to_vendor(mac)
    return _vendor_to_url(vendor, ip=ip, substream=substream)
