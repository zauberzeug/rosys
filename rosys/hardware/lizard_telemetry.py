"""Decoder for Lizard's telemetry frames (see "Telemetry frames" in Lizard's docs/module_reference.md).

Copied from Lizard's ``lizard_host/telemetry.py`` (zauberzeug/lizard@0b78ad6), since ``lizard_host`` is not an
installable package; unchanged except for this docstring and the lint exceptions below.

A frame line is "~<base64>" with the body frame ID, layout ID, sequence number, millis, payload and CRC-16;
layout lines "__LAYOUT__v1 <frame>.<index>/<count> <layout ID> <name>:<type>" name and type its fields.
"""
# ruff: noqa: UP006, UP007, UP035, UP045  # Lizard's host tools keep the typing of Python 3.8
# pylint: disable=deprecated-class
import base64
import binascii
import re
import struct
from dataclasses import dataclass
from typing import Dict, Hashable, List, Literal, Optional, Tuple, Union, cast

VERSION = 'v1'
HEADER = struct.Struct('<BBBI')  # frame ID, layout ID, sequence number, millis
CRC_SIZE = 2
LAYOUT_LINE = re.compile(r'__LAYOUT__(v\d+)(.*)')
LAYOUT_FIELD = re.compile(r' (\d+)\.(\d+)/(\d+) ([0-9a-f]{2}) (\S+):([fi?])')
BUS_LINE = re.compile(r'bus\[(\d+)\]: (.*)')  # a line that a serial bus coordinator passes on from peer N

Value = Union[float, int, bool]
Field = Tuple[str, str]  # name and type
Status = Literal['ok', 'no_layout', 'layout_incomplete', 'layout_mismatch', 'unknown_version']


def format_value(value: Value) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, float):
        return f'{value:.7g}'  # the significant digits of a float32, without noise
    return str(value)


@dataclass
class Text:
    sender: Hashable
    line: str


@dataclass
class Layout:
    sender: Hashable
    frame_id: int
    layout_id: int
    index: int
    count: int
    name: str
    type: str


@dataclass
class Frame:
    sender: Hashable
    frame_id: int
    layout_id: int
    seq: int
    millis: int
    status: Status
    values: Optional[Dict[str, Value]]  # in field order, None unless the status is ok
    payload: bytes

    def __str__(self) -> str:
        head = f'frame {self.frame_id} seq={self.seq} millis={self.millis}'
        if self.values is None:
            return f'[{head} {self.status}] {self.payload.hex()}'
        return f'[{head}] ' + ' '.join(f'{name}={format_value(value)}' for name, value in self.values.items())


class TelemetryDecoder:
    """Turns console lines into Text, Layout and Frame results and keeps the layouts of each sender.

    The sender is the device on the console, or serial bus peer N for a line starting with "bus[N]: ".
    """

    def __init__(self) -> None:
        self._versions: Dict[Hashable, str] = {}  # format version of the sender's last layout line
        self._layouts: Dict[Tuple[Hashable, int], Tuple[int, List[Optional[Field]]]] = {}  # layout ID and fields

    def feed(self, line: str, sender: Hashable = None) -> Union[Text, Layout, Frame]:
        """Classify a line given without line end and checksum.

        The layout lines and frames of a line "bus[N]: ..." come from peer N,
        so their sender is N or, if a sender is given, (sender, N).
        """
        relayed = BUS_LINE.fullmatch(line)
        if relayed:
            peer = int(relayed.group(1))
            result = self._telemetry(relayed.group(2), peer if sender is None else (sender, peer))
        else:
            result = self._telemetry(line, sender)
        return result or Text(sender, line)

    def _telemetry(self, line: str, sender: Hashable) -> Union[Layout, Frame, None]:
        if line.startswith('~'):
            body = _frame_body(line[1:])
            return None if body is None else self._frame(sender, body)
        match = LAYOUT_LINE.fullmatch(line)
        if match is None:
            return None
        version, rest = match.groups()
        if version != VERSION:
            self._set_version(sender, version)
            return None
        return self._layout(sender, rest)

    def _set_version(self, sender: Hashable, version: str) -> None:
        if self._versions.get(sender) != version:  # layouts of another format version must not decode frames
            self._versions[sender] = version
            for key in [key for key in self._layouts if key[0] == sender]:
                del self._layouts[key]

    def _layout(self, sender: Hashable, rest: str) -> Optional[Layout]:
        match = LAYOUT_FIELD.fullmatch(rest)
        if match is None:
            return None
        frame_id, index, count = int(match.group(1)), int(match.group(2)), int(match.group(3))
        if index >= count:
            return None
        self._set_version(sender, VERSION)
        layout_id = int(match.group(4), 16)
        field = (match.group(5), match.group(6))
        key = (sender, frame_id)
        known_id, fields = self._layouts.get(key, (None, []))
        if known_id != layout_id or len(fields) != count or fields[index] not in (None, field):
            # another layout for the frame ID, or other fields that happen to get the same layout ID
            fields = [None] * count
            self._layouts[key] = (layout_id, fields)
        fields[index] = field
        return Layout(sender, frame_id, layout_id, index, count, *field)

    def _frame(self, sender: Hashable, body: bytes) -> Frame:
        frame_id, layout_id, seq, millis = HEADER.unpack_from(body)
        payload = body[HEADER.size:-CRC_SIZE]
        if self._versions.get(sender, VERSION) != VERSION:
            return Frame(sender, frame_id, layout_id, seq, millis, 'unknown_version', None, payload)
        known_id, fields = self._layouts.get((sender, frame_id), (None, []))
        if known_id != layout_id:  # also when the frame ID was reused for other fields
            return Frame(sender, frame_id, layout_id, seq, millis, 'no_layout', None, payload)
        if None in fields:
            return Frame(sender, frame_id, layout_id, seq, millis, 'layout_incomplete', None, payload)
        values = _decode(cast(List[Field], fields), payload)
        status: Status = 'layout_mismatch' if values is None else 'ok'
        return Frame(sender, frame_id, layout_id, seq, millis, status, values, payload)


def _frame_body(text: str) -> Optional[bytes]:
    try:
        body = base64.b64decode(text, validate=True)
    except ValueError:  # includes binascii.Error and non-ASCII text
        return None
    if len(body) < HEADER.size + CRC_SIZE:
        return None
    if binascii.crc_hqx(body[:-CRC_SIZE], 0xffff) != int.from_bytes(body[-CRC_SIZE:], 'little'):  # CCITT-FALSE
        return None
    return body


def _decode(layout: List[Field], payload: bytes) -> Optional[Dict[str, Value]]:
    numbers = sum(type_ != '?' for _, type_ in layout)
    bits_offset = 4 * numbers  # the bools follow the floats and ints as bits
    if len(payload) != bits_offset + (len(layout) - numbers + 7) // 8:
        return None
    values: Dict[str, Value] = {}
    offset = bit = 0
    for name, type_ in layout:
        if type_ == '?':
            values[name] = bool(payload[bits_offset + bit // 8] >> bit % 8 & 1)
            bit += 1
        else:
            values[name] = struct.unpack_from('<' + type_, payload, offset)[0]
            offset += 4
    return values
