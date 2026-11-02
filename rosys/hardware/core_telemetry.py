import logging
import math
import re
from collections.abc import Callable

from .. import rosys
from .lizard_telemetry import Frame, TelemetryDecoder, Value

MAX_PAYLOAD = 180  # bytes of values in one frame, Lizard's telemetry::MAX_PAYLOAD
FIELD_SIZE = 4  # bytes counted per field: the host does not know the types, so each counts as a float or int
LINE_LIMIT = 1800  # characters of one core.telemetry(...) line, well within Lizard's console line of 2048 bytes
LAYOUT_REQUEST_INTERVAL = 2.0  # seconds between two core.telemetry_info() requests for missing layouts
WARNING_INTERVAL = 30.0  # seconds between two notifications about frames of an outdated startup script
FIELD_NAME = re.compile(r'[A-Za-z_]\w*(\.[A-Za-z_]\w*){0,2}')  # variable, module.property or module.property.sub


class CoreTelemetry:
    """Defines the core's telemetry frames and turns them back into the text line ``core.output`` would have printed.

    The fields are the modules' ``core_message_fields``: properties and variables with an optional ``:<precision>``.
    The frames carry each field once and without its precision; the line repeats and formats them like ``core.output``.
    Floats travel as float32, so beyond about 7 significant digits the line differs from that of ``core.output``.
    Fields beyond one frame go into several; the line is built once all frames with the same millis have arrived.
    A frame of other fields is taken as a sign of an outdated startup script.
    """

    def __init__(self, request_layouts: Callable[[], None]) -> None:
        """
        :param request_layouts: called to send ``core.telemetry_info()``, at most every ``LAYOUT_REQUEST_INTERVAL``
        """
        self._request_layouts = request_layouts
        self._decoder = TelemetryDecoder()
        self._fields: list[tuple[str, int]] = []  # name and precision, in the order of the text line
        self._frames: set[tuple[str, ...]] = set()
        self._last_request = -math.inf
        self._last_warning = -math.inf
        self._step_millis: int | None = None
        self._step_values: dict[str, Value] = {}
        self._step_frames: set[tuple[str, ...]] = set()

    def lizard_code(self, output_fields: list[str]) -> str:
        """Plan the frames for the given ``core.output`` fields and return the Lizard code that defines them.

        :raises ValueError: if a field is not a property or variable with an optional precision
        """
        self._fields = [_parse(field) for field in output_fields]
        frames: list[list[str]] = [[]]
        for name in dict.fromkeys(name for name, _ in self._fields):
            if frames[-1] and not _fits([*frames[-1], name]):
                frames.append([])
            frames[-1].append(name)
        if not frames[0]:
            frames = [['core.millis']]  # Lizard needs a field, and the frame still marks each step
        self._frames = {tuple(frame) for frame in frames}
        self._step_millis = None
        return ''.join(f'{_code(frame)}\n' for frame in frames)

    def translate(self, line: str) -> str:
        """Translate a console line of the core, given without checksum.

        :return: the line itself unless it is a telemetry line (``~...`` or ``__LAYOUT__...``);
            for those the text line once all frames of a step have arrived, otherwise an empty line
        """
        if not line.startswith(('~', '__LAYOUT__')):
            return line
        frame = self._decoder.feed(line)
        if not isinstance(frame, Frame):  # a layout line or a malformed line
            return ''
        if frame.values is None:
            self._request_layouts_throttled()
            return ''
        fields = tuple(frame.values)
        if fields not in self._frames:
            self._warn_throttled()
            return ''
        if frame.millis != self._step_millis:
            self._step_millis = frame.millis
            self._step_values = {}
            self._step_frames = set()
        self._step_values.update(frame.values)
        self._step_frames.add(fields)
        if self._step_frames != self._frames:
            return ''
        self._step_millis = None  # the next frame starts a new step, even with the same millis
        words = [_format(self._step_values[name], precision) for name, precision in self._fields]
        return ' '.join(['core', str(frame.millis), *words])

    def _request_layouts_throttled(self) -> None:
        now = rosys.time()
        if now - self._last_request >= LAYOUT_REQUEST_INTERVAL:
            self._last_request = now
            self._request_layouts()

    def _warn_throttled(self) -> None:
        now = rosys.time()
        if now - self._last_warning >= WARNING_INTERVAL:
            self._last_warning = now
            rosys.notify('Lizard startup code is outdated. Please configure.', 'negative', log_level=logging.WARNING)


def _parse(field: str) -> tuple[str, int]:
    name, _, precision = field.partition(':')
    if not FIELD_NAME.fullmatch(name) or (precision and not precision.isdigit()):
        raise ValueError(f'core telemetry field "{field}" is not a property or variable with an optional precision')
    return name, int(precision or 0)


def _code(frame: list[str]) -> str:
    return f'core.telemetry({", ".join(frame)})'


def _fits(frame: list[str]) -> bool:
    return len(frame) * FIELD_SIZE <= MAX_PAYLOAD and len(_code(frame)) <= LINE_LIMIT


def _format(value: Value, precision: int) -> str:
    """A value as ``core.output`` prints it, floats with the field's precision."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, int):
        return str(value)
    return f'{value:.{precision}f}'
