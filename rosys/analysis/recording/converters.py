"""Registry, topic sources and JSON machinery that feed the MCAP recorder.

A :class:`Converter` bundles a :class:`TopicSchema` with an ``encode`` returning
JSON bytes (or ``None`` to skip). The registry maps RoSys types to converter
factories so :func:`add_event_topic` / :func:`add_timer_topic` pick the right
converter from the value's runtime type, with explicit overrides. App-specific
topics without a foxglove schema use :func:`custom_message` with a JSON schema.

The concrete converters for RoSys/Foxglove data types live in :mod:`.foxglove`;
this module only holds the machinery that wires values into the recorder. Values
are enqueued unencoded and :func:`Converter.encode` runs on the recorder's
background writer, so no JSON/JPEG encoding happens on the event loop.
"""
from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from ... import rosys
from .mcap_recorder import NANOSECONDS_PER_SECOND, McapRecorder, TopicSchema

if TYPE_CHECKING:
    from nicegui import Event

    from ...rosys import Repeater

# --------------------------------------------------------------------------- #
# Converter definition and registry
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class Converter:
    """A topic's schema plus an ``encode`` returning JSON bytes (or ``None`` to skip)."""
    schema: TopicSchema
    encode: Callable[[Any, int], bytes | None]


ConverterFactory = Callable[..., Converter]

_REGISTRY: dict[type, ConverterFactory] = {}


def register(data_type: type, factory: ConverterFactory) -> None:
    """Associate a RoSys data type with a converter factory for auto-dispatch."""
    _REGISTRY[data_type] = factory


def converter_for(value: Any, **overrides) -> Converter:
    """Resolve the converter for a value via its type's MRO, applying overrides."""
    for data_type in type(value).__mro__:
        factory = _REGISTRY.get(data_type)
        if factory is not None:
            return factory(**overrides)
    raise TypeError(f'no converter registered for {type(value).__name__}; '
                    f'pass converter=... explicitly')


# --------------------------------------------------------------------------- #
# Topic registration: event- or timer-driven
# --------------------------------------------------------------------------- #


def add_event_topic(recorder: McapRecorder, topic: str, *,
                    event: Event,
                    converter: Converter | None = None,
                    unpack: bool = False,
                    argless: bool = False,
                    timestamp: Callable[[Any], float] | None = None,
                    **overrides) -> None:
    """Record a topic fed by an event.

    The converter is auto-picked from the event payload's type unless given
    explicitly. ``overrides`` are forwarded to the auto-picked converter factory
    (e.g. ``frame='map'``). With ``unpack=True`` the payload is treated as an
    iterable and each element is recorded separately (e.g. ``list[Velocity]``).
    ``argless=True`` is for events that emit no payload (the converter is fed
    ``None``); it must then be explicit. ``timestamp`` overrides the log time
    (seconds); by default the payload's ``time`` attribute is used, falling back
    to ``rosys.time()``. The subscription is only active while recording.
    """
    if converter is not None:
        recorder.add_topic(topic, converter.schema)
    else:
        recorder.declare_topic(topic)
    recorder.add_source(_EventSource(recorder, topic, event, converter, overrides, unpack, argless, timestamp))


def add_timer_topic(recorder: McapRecorder, topic: str, *,
                    interval: float,
                    read: Callable[[], Any],
                    converter: Converter | None = None,
                    timestamp: Callable[[Any], float] | None = None,
                    **overrides) -> None:
    """Record a topic by polling ``read`` every ``interval`` seconds.

    A ``read`` returning ``None`` is skipped (source not ready yet). The
    converter is auto-picked from the returned value's type unless given
    explicitly. ``timestamp`` overrides the log time (seconds). The timer only
    runs while the recorder is recording.
    """
    if converter is not None:
        recorder.add_topic(topic, converter.schema)
    else:
        recorder.declare_topic(topic)
    recorder.add_source(_TimerSource(recorder, topic, interval, read, converter, overrides, timestamp))


class _TopicWriter:
    """Shared lazy-dispatch + enqueue for a single topic.

    Auto-dispatch and timestamping happen on the event loop (both cheap), but the
    value is handed to the recorder unencoded together with its ``encode``
    callable, so the actual JSON/JPEG encoding runs on the background writer.
    """

    def __init__(self, recorder: McapRecorder, topic: str, converter: Converter | None,
                 overrides: dict, timestamp: Callable[[Any], float] | None) -> None:
        self._recorder = recorder
        self._topic = topic
        self._converter = converter
        self._overrides = overrides
        self._timestamp = timestamp

    def write(self, value: Any) -> None:
        if not self._recorder.accepts(self._topic):
            return  # not recording or topic deselected -> skip before any (auto-dispatch or encode) work
        if self._converter is None:
            if value is None:
                return  # cannot auto-dispatch a None payload
            self._converter = converter_for(value, **self._overrides)
            self._recorder.add_topic(self._topic, self._converter.schema)
        timestamp_ns = int(self._timestamp(value) * NANOSECONDS_PER_SECOND) \
            if self._timestamp is not None else _timestamp_ns(value)
        self._recorder.log_message(self._topic, value, encode=self._converter.encode, timestamp_ns=timestamp_ns)


class _EventSource:

    def __init__(self, recorder: McapRecorder, topic: str, event: Event, converter: Converter | None,
                 overrides: dict, unpack: bool, argless: bool,
                 timestamp: Callable[[Any], float] | None) -> None:
        self._event = event
        self._unpack = unpack
        self._writer = _TopicWriter(recorder, topic, converter, overrides, timestamp)
        # NOTE: nicegui passes the emitted payload only if the handler has a required
        # positional parameter; an argless event needs a parameterless handler instead.
        self._handler = self._handle_argless if argless else self._handle

    def start(self) -> None:
        # unsubscribe_on_delete=False: the recorder outlives any browser tab, so the
        # subscription must not be torn down when the client that started recording
        # (e.g. via the developer panel) disconnects. Otherwise event-driven topics
        # would silently stop being recorded when that tab closes.
        self._event.subscribe(self._handler, unsubscribe_on_delete=False)

    def stop(self) -> None:
        self._event.unsubscribe(self._handler)

    def _handle(self, payload: Any, *_args) -> None:
        items = payload if self._unpack else [payload]
        if items is None:
            return
        for value in items:
            self._writer.write(value)

    def _handle_argless(self) -> None:
        self._writer.write(None)


class _TimerSource:

    def __init__(self, recorder: McapRecorder, topic: str, interval: float, read: Callable[[], Any],
                 converter: Converter | None, overrides: dict,
                 timestamp: Callable[[Any], float] | None) -> None:
        self._interval = interval
        self._read = read
        self._writer = _TopicWriter(recorder, topic, converter, overrides, timestamp)
        self._repeater: Repeater | None = None

    def start(self) -> None:
        self._repeater = rosys.on_repeat(self._poll, self._interval)

    def stop(self) -> None:
        if self._repeater is not None:
            self._repeater.stop()
            self._repeater = None

    def _poll(self) -> None:
        value = self._read()
        if value is not None:
            self._writer.write(value)


def _timestamp_ns(value: Any) -> int:
    time = getattr(value, 'time', None)
    if isinstance(time, (int, float)) and time > 0:
        return int(time * NANOSECONDS_PER_SECOND)
    return int(rosys.time() * NANOSECONDS_PER_SECOND)


# --------------------------------------------------------------------------- #
# JSON machinery
# --------------------------------------------------------------------------- #


def custom_message(schema_name: str, schema: dict, build: Callable[[Any, int], dict | None]) -> Converter:
    """Converter for an app-specific JSON message; ``build`` returns a dict (or None to skip)."""
    topic_schema = TopicSchema(schema_name, json.dumps(schema).encode(), 'jsonschema', 'json')

    def encode(value: Any, timestamp_ns: int) -> bytes | None:
        message = build(value, timestamp_ns)
        if message is None:
            return None
        return json.dumps(message, default=_json_native).encode()

    return Converter(topic_schema, encode)


def _json_native(obj: Any) -> Any:
    """Coerce non-serializable values to native Python (e.g. numpy bool_/int64 -> bool/int)."""
    item = getattr(obj, 'item', None)  # numpy scalars expose .item()
    if callable(item):
        return item()
    raise TypeError(f'Object of type {type(obj).__name__} is not JSON serializable')


# --------------------------------------------------------------------------- #
# Plottable scalar converters (no foxglove well-known schema)
# --------------------------------------------------------------------------- #


# A distinct schema name per value type: Foxglove keys datatypes by name, so a
# shared 'Scalar' name with differing `value` types (e.g. float64 vs bool) would
# collide on load. The name still groups same-typed scalars across topics.
_SCALAR_SCHEMA_NAME = {'number': 'Float', 'integer': 'Integer', 'boolean': 'Boolean', 'string': 'String'}


def scalar(*, value_type: str = 'number', extract: Callable[[Any], Any] | None = None) -> Converter:
    """A primitive value -> ``{value: ...}`` (plottable). ``extract`` pulls the field off the payload."""
    schema = {'type': 'object', 'properties': {'value': {'type': value_type}}}

    def build(value: Any, _timestamp_ns: int) -> dict:
        return {'value': extract(value) if extract is not None else value}

    return custom_message(_SCALAR_SCHEMA_NAME[value_type], schema, build)


def number() -> Converter:
    """``float`` -> ``{value: <number>}``."""
    return scalar(value_type='number')


def integer() -> Converter:
    """``int`` -> ``{value: <integer>}``."""
    return scalar(value_type='integer')


def boolean() -> Converter:
    """``bool`` -> ``{value: <boolean>}``."""
    return scalar(value_type='boolean')


def text() -> Converter:
    """``str`` -> ``{value: <string>}``."""
    return scalar(value_type='string')


register(bool, boolean)  # before int: bool is a subclass of int
register(int, integer)
register(float, number)
register(str, text)
