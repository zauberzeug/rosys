"""The core's telemetry as Lizard telemetry frames instead of the ``core.output`` text line.

With ``RobotBrain(telemetry_frames=True)`` the generated startup defines ``core.telemetry(core.millis, <fields>, 0)``
instead of ``core.output("core.millis <fields>")``, and :class:`CoreTelemetry` turns the frames back into the text line
``core <millis> <values...>`` that ``core.output`` would have printed, so the modules' ``handle_core_output`` and the
robot brain's handling of core lines stay as they are.
"""
import logging
import re
from collections.abc import Callable
from dataclasses import dataclass

from .. import rosys
from .lizard_telemetry import Frame, Layout, TelemetryDecoder, Value

MILLIS = 'core.millis'
MAX_PAYLOAD = 180
"""Bytes of values in one frame (Lizard's ``telemetry::MAX_PAYLOAD``)."""
FIELD_SIZE = 4
"""Bytes the plan counts per field: the host does not know the fields' types, and a float or int takes 4 bytes."""
LINE_LIMIT = 1800
"""Characters of one ``core.telemetry(...)`` startup line, well within Lizard's console line of 2048 bytes."""
LAYOUT_REQUEST_INTERVAL = 2.0
"""Seconds between two ``core.telemetry_info()`` requests for layouts the host is missing."""
WARNING_INTERVAL = 30.0
FIELD_NAME = re.compile(r'[A-Za-z_]\w*(\.[A-Za-z_]\w*){0,2}')

log = logging.getLogger('rosys.core_telemetry')


@dataclass(frozen=True, slots=True)
class _Field:
    name: str
    precision: int | None


class CoreTelemetry:
    """Plans the core's telemetry frames and turns them back into the core's text line.

    The fields are the ``core_message_fields`` of the modules, so they are properties and variables, possibly with a
    ``:<precision>`` suffix; the frames carry them without it, and the text line formats floats with it, like
    ``core.output`` does. A field appears once per frame even if several modules stream it.
    Fields that do not fit into one frame go into several, each starting with ``core.millis``; the host collects the
    frames of one step by their millis and builds the line once all of them arrived.
    """

    def __init__(self, request_layouts: Callable[[], None], *, clock: Callable[[], float] = rosys.time) -> None:
        """
        :param request_layouts: called to send ``core.telemetry_info()``, at most every ``LAYOUT_REQUEST_INTERVAL``
        :param clock: the time in seconds, for the request interval
        """
        self._request_layouts = request_layouts
        self._clock = clock
        self._decoder = TelemetryDecoder()
        self._fields: list[_Field] = []
        self._frames: dict[tuple[str, ...], int] = {}
        self._last_request: float | None = None
        self._last_warning: float | None = None
        self._step_millis: Value | None = None
        self._step_values: dict[str, Value] = {}
        self._step_frames: set[int] = set()
        self.frames_without_layout = 0
        """Frames dropped because their layout lines had not arrived (yet)."""
        self.unexpected_frames = 0
        """Frames dropped because their fields are not those of the planned frames, e.g. of an outdated startup."""
        self.incomplete_steps = 0
        """Steps dropped because one of their frames got lost."""
        self.malformed_lines = 0
        """Telemetry lines that could not be decoded, e.g. with a wrong CRC."""

    @property
    def frames(self) -> list[list[str]]:
        """The planned frames, as field names in order."""
        return [list(names) for names in self._frames]

    def plan(self, output_fields: list[str]) -> str:
        """Plan the frames for the given ``core.output`` fields and return the Lizard code defining them.

        :raises ValueError: if a field is no plain property or variable, or too long for a frame line
        """
        fields = [_parse(field) for field in output_fields]
        names = list(dict.fromkeys(field.name for field in fields if field.name != MILLIS))
        frames: list[list[str]] = []
        chunk: list[str] = []
        for name in names:
            if chunk and not _fits([MILLIS, *chunk, name]):
                frames.append([MILLIS, *chunk])
                chunk = []
            chunk.append(name)
        frames.append([MILLIS, *chunk])
        for frame in frames:
            if not _fits(frame):
                raise ValueError(f'core telemetry field too long for a frame line: {frame[-1][:60]}')
        self._fields = fields
        self._frames = {tuple(frame): index for index, frame in enumerate(frames)}
        self._reset_step()
        return ''.join(f'{_code(frame)}\n' for frame in frames)

    def translate(self, line: str) -> str | None:
        """Consume a telemetry line of the core (``~...`` or ``__LAYOUT__...``, without checksum).

        :return: the core's text line once all frames of a step arrived, otherwise ``None``
        """
        result = self._decoder.feed(line)
        if isinstance(result, Layout):
            return None
        if not isinstance(result, Frame):
            self.malformed_lines += 1
            return None
        if result.values is None:
            self.frames_without_layout += 1
            self._request_layouts_throttled()
            return None
        index = self._frames.get(tuple(result.values))
        if index is None:
            self.unexpected_frames += 1
            self._warn_throttled(f'Lizard telemetry frame {result.frame_id} does not carry the configured fields, '
                                 'the core probably runs an outdated startup; please configure')
            return None
        millis = result.values[MILLIS]
        if millis != self._step_millis:
            if self._step_frames:
                self.incomplete_steps += 1
            self._reset_step()
            self._step_millis = millis
        self._step_values.update(result.values)
        self._step_frames.add(index)
        if len(self._step_frames) < len(self._frames):
            return None
        values = self._step_values
        self._reset_step()
        words = [_format(values[MILLIS], None), *(_format(values[field.name], field.precision)
                                                  for field in self._fields)]
        return 'core ' + ' '.join(words)

    def _reset_step(self) -> None:
        self._step_millis = None
        self._step_values = {}
        self._step_frames = set()

    def _request_layouts_throttled(self) -> None:
        now = self._clock()
        if self._last_request is not None and now - self._last_request < LAYOUT_REQUEST_INTERVAL:
            return
        self._last_request = now
        self._request_layouts()

    def _warn_throttled(self, message: str) -> None:
        now = self._clock()
        if self._last_warning is not None and now - self._last_warning < WARNING_INTERVAL:
            return
        self._last_warning = now
        rosys.notify(message, 'negative', log_level=logging.ERROR)


def _parse(field: str) -> _Field:
    name, _, precision = field.partition(':')
    if not FIELD_NAME.fullmatch(name) or (precision and not precision.isdigit()):
        raise ValueError(f'core telemetry field "{field}" is no property or variable with an optional precision')
    return _Field(name, int(precision) if precision else None)


def _code(frame: list[str]) -> str:
    return f'core.telemetry({", ".join(frame)}, 0)'


def _fits(frame: list[str]) -> bool:
    return len(frame) * FIELD_SIZE <= MAX_PAYLOAD and len(_code(frame)) <= LINE_LIMIT


def _format(value: Value, precision: int | None) -> str:
    """A value as ``core.output`` prints it: floats with the field's precision, which defaults to 0."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, int):
        return str(value)
    return f'{value:.{precision or 0}f}'
