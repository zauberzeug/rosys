import logging
import math
import re
from collections.abc import Callable

import lizard_host as lh

from .. import rosys

LINE_LIMIT = 1800  # characters of one core.telemetry(...) line, well within Lizard's console line of 2048 bytes
WARNING_INTERVAL = 30.0  # seconds between two notifications about frames of an outdated startup script
OUTDATED_AFTER_STEPS = 300  # steps in a row without all fields (3 s of Lizard's 10 ms loop) that count as outdated
FIELD_NAME = re.compile(r'[A-Za-z_]\w*(\.[A-Za-z_]\w*){0,2}')  # variable, module.property or module.property.sub


class CoreTelemetry:
    """Defines the core's telemetry frames and turns them back into the text line ``core.output`` would have printed.

    The fields are the modules' ``core_message_fields``: properties and variables with an optional ``:<precision>``.
    ``core.telemetry`` gets each field once and without its precision and splits them into frames by itself;
    the line repeats and formats them like ``core.output``.
    Floats travel as float32, so beyond about 7 significant digits the line differs from that of ``core.output``.
    The frames of a step carry the same millis, and the line is built as soon as they brought all fields.
    A frame with another field, and frames that keep lacking fields, are taken as a sign of an outdated startup script.
    """

    def __init__(self, request_layouts: Callable[[], None]) -> None:
        """
        :param request_layouts: called to send ``core.telemetry_info()`` when a frame lacks its layout
        """
        self._request_layouts = request_layouts
        self._decoder = lh.TelemetryDecoder()
        self._layout_requester = lh.LayoutRequester()
        self._fields: list[tuple[str, int]] = []  # name and precision, in the order of the text line
        self._names: frozenset[str] = frozenset()
        self._step: lh.Step | None = None  # the frames that arrived with the current millis
        self._incomplete_steps = 0
        self._last_warning = -math.inf

    def lizard_code(self, output_fields: list[str]) -> str:
        """Take the given ``core.output`` fields and return the Lizard code that sends them as telemetry frames.

        Lizard splits a call into as many frames as its values need, so the fields only go into several calls
        where a line of the startup script would get too long.

        :raises ValueError: if a field is not a property or variable with an optional precision
        """
        self._fields = [_parse(field) for field in output_fields]
        # Lizard needs a field, and the frame still marks each step
        names = list(dict.fromkeys(name for name, _ in self._fields)) or ['core.millis']
        calls: list[list[str]] = [[]]
        for name in names:
            if calls[-1] and len(_code([*calls[-1], name])) > LINE_LIMIT:
                calls.append([])
            calls[-1].append(name)
        self._names = frozenset(names)
        self._step = None
        self._incomplete_steps = 0
        return ''.join(f'{_code(call)}\n' for call in calls)

    def translate(self, line: str) -> str:
        """Translate a console line of the core, given without checksum.

        :return: the line itself unless it is a telemetry line (``~...`` or ``__LAYOUT__...``);
            for those the text line once all fields of a step have arrived, otherwise an empty line
        """
        if not line.startswith(('~', '__LAYOUT__')):
            return line
        frame = self._decoder.feed(line)
        if self._layout_requester.feed(frame, rosys.time()):
            self._request_layouts()
        if not isinstance(frame, lh.Frame) or frame.values is None:
            return ''  # a layout line, a malformed line or a frame without its layout
        if not self._names.issuperset(frame.values):
            self._warn_throttled()
            return ''
        if self._step is None or self._step.millis != frame.millis:
            if self._step is not None:  # the step before never brought all fields
                self._incomplete_steps += 1
                if self._incomplete_steps >= OUTDATED_AFTER_STEPS:
                    self._warn_throttled()
            self._step = lh.Step(frame.sender, frame.millis, [])
        self._step.frames.append(frame)
        values = self._step.values
        if len(values) < len(self._names):
            return ''
        self._step = None  # the next frame starts a new step, even with the same millis
        self._incomplete_steps = 0
        words = [_format(values[name], precision) for name, precision in self._fields]
        return ' '.join(['core', str(frame.millis), *words])

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


def _code(names: list[str]) -> str:
    return f'core.telemetry({", ".join(names)})'


def _format(value: float | int | bool, precision: int) -> str:
    """A value as ``core.output`` prints it, floats with the field's precision."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, int):
        return str(value)
    return f'{value:.{precision}f}'
