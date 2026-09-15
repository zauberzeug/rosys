import enum
import math
import pickle
import struct
from dataclasses import dataclass

import numpy as np

from ..image import ImageArray


class EndReason(enum.Enum):
    ENDED = enum.auto()
    REFUSED = enum.auto()  # the camera answered something other than a stream
    UNREACHABLE = enum.auto()  # http error
    STALLED = enum.auto()  # the camera stopped sending data
    FAILED = enum.auto()  # anything else, including the worker dying


@dataclass(slots=True, kw_only=True)
class Frame:
    array: ImageArray
    capture_time: float | None


@dataclass(slots=True, kw_only=True)
class StreamEnded:
    reason: EndReason
    detail: str = ''


Message = Frame | StreamEnded

_FRAME_TAG = 0
_END_TAG = 1
_FRAME_HEADER = struct.Struct('<BIIId')  # tag, height, width, channels, capture time (NaN when the camera sent none)


def encode(message: Message) -> bytes | bytearray:
    """Pack a message into the single buffer a conflating socket can carry, copying the pixels once."""
    if not isinstance(message, Frame):
        return bytes([_END_TAG]) + pickle.dumps(message)
    array = np.ascontiguousarray(message.array)
    height, width, channels = array.shape
    buffer = bytearray(_FRAME_HEADER.size + array.nbytes)
    _FRAME_HEADER.pack_into(buffer, 0, _FRAME_TAG, height, width, channels,
                            math.nan if message.capture_time is None else message.capture_time)
    buffer[_FRAME_HEADER.size:] = array.data
    return buffer


def decode(buffer: memoryview) -> Message:
    """Unpack a message; a frame's pixels stay in the given buffer instead of being copied out of it."""
    if buffer[0] == _END_TAG:
        return pickle.loads(buffer[1:])
    _, height, width, channels, capture_time = _FRAME_HEADER.unpack_from(buffer)
    array = np.frombuffer(buffer, dtype=np.uint8, offset=_FRAME_HEADER.size).reshape(height, width, channels)
    return Frame(array=array, capture_time=None if math.isnan(capture_time) else capture_time)
