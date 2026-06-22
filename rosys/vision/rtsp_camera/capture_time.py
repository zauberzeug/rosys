"""Recover the absolute capture instant of a frame from an RTP header extension.

Cameras that stamp frames at capture time carry the instant in the WebRTC
"Absolute Capture Time" RTP header extension: a 64-bit Q32.32 NTP timestamp
(seconds since 1900-01-01, big-endian) on the first packet of each frame.
Unlike the RTP timestamp it is self-anchored -- each frame carries a freshly
clock-corrected absolute time -- so it needs neither an RTCP Sender Report
anchor nor extrapolation, while the RTP timestamp stays a clean media clock.

See http://www.webrtc.org/experiments/rtp-hdrext/abs-capture-time.
"""
from __future__ import annotations

from typing import Any

ABS_CAPTURE_TIME_URI = 'http://www.webrtc.org/experiments/rtp-hdrext/abs-capture-time'

# Seconds between the 1900 (NTP) and 1970 (Unix) epochs.
_NTP_UNIX_EPOCH_OFFSET = 2_208_988_800

# Default one-byte header-extension id for abs-capture-time. The id is properly negotiated
# via the SDP `a=extmap` line; until that is parsed the extension is read at this fixed id.
DEFAULT_ABS_CAPTURE_TIME_EXT_ID = 10


def decode_abs_capture_time(payload: bytes) -> float:
    """Decode the extension payload (>=8 bytes; trailing optional fields ignored) to Unix epoch seconds."""
    return int.from_bytes(payload[:8], 'big') / (1 << 32) - _NTP_UNIX_EPOCH_OFFSET


def encode_abs_capture_time(epoch: float) -> bytes:
    """Encode a Unix epoch (seconds) as the extension's 8-byte Q32.32 NTP payload (inverse of decode)."""
    q32_32 = round((epoch + _NTP_UNIX_EPOCH_OFFSET) * (1 << 32)) & ((1 << 64) - 1)
    return q32_32.to_bytes(8, 'big')


def read_abs_capture_time(rtp_buffer: Any, ext_id: int = DEFAULT_ABS_CAPTURE_TIME_EXT_ID) -> float | None:
    """Read the abs-capture-time epoch (s) from a mapped ``GstRtp.RTPBuffer`` one-byte header extension.

    Returns ``None`` when the extension is absent, so the caller can fall back to the receive time.
    ``rtp_buffer`` is kept untyped to avoid importing the GStreamer bindings at module import time.
    """
    ok, data = rtp_buffer.get_extension_onebyte_header(ext_id, 0)
    if not ok or not data:
        return None
    return decode_abs_capture_time(bytes(data))
