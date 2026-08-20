from __future__ import annotations

import asyncio
import re
import struct
from dataclasses import dataclass
from enum import Enum

# See https://maemo.org/api_refs/5.0/5.0-final/gstreamer-libs-0.10/gstreamer-libs-gstdataprotocol.html for header format
GDPPACKET_FORMAT = struct.Struct('>HcxHIQQQQH14sHH')
GDP_CAPS_WIDTH_REGEX = re.compile(r'width=\(int\)\s*(\d+)')
GDP_CAPS_HEIGHT_REGEX = re.compile(r'height=\(int\)\s*(\d+)')
GDP_HEADER_SIZE = 62


class GDPPayloadType(Enum):
    NONE = 0
    BUFFER = 1
    CAPS = 2
    EVENT_NONE = 3


@dataclass(slots=True, kw_only=True)
class GDPPacket:
    """A single packet of the GStreamer Data Protocol as emitted by a `gdppay ! fdsink` pipeline."""
    payload_type: GDPPayloadType
    payload: bytes

    @staticmethod
    async def read(stream: asyncio.StreamReader) -> GDPPacket:
        header_bytes = await stream.readexactly(GDP_HEADER_SIZE)
        _version, _flags, gdp_type, length, *_ = GDPPACKET_FORMAT.unpack(header_bytes)
        return GDPPacket(
            payload_type=GDPPayloadType(gdp_type) if gdp_type < 3 else GDPPayloadType.EVENT_NONE,
            payload=await stream.readexactly(length),
        )


def parse_caps_dimensions(cap_text: str) -> tuple[int, int]:
    """Extract width and height from the text of a GDP CAPS packet."""
    width_match = GDP_CAPS_WIDTH_REGEX.search(cap_text)
    height_match = GDP_CAPS_HEIGHT_REGEX.search(cap_text)
    if width_match is None or height_match is None:
        raise ValueError(f'could not parse width and height from caps: {cap_text}')
    return int(width_match.group(1)), int(height_match.group(1))
