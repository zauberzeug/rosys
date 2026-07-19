from __future__ import annotations

import logging
import threading
from collections.abc import Callable, Collection
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, BinaryIO, NamedTuple, Protocol

from mcap.writer import CompressionType, Writer
from nicegui import Event, ui

from ... import rosys
from .indexing import is_indexed, reindex
from .paths import PAGE_PATH

NANOSECONDS_PER_SECOND = 1_000_000_000

MAX_QUEUED_MESSAGES = 10_000
"""Cap on unwritten messages held in memory.

At the mix of high-rate topics a robot records (camera frames plus sensors) this is
a few seconds of backlog, which comfortably absorbs a slow flush; if the disk
stalls for longer the oldest messages are dropped (see :meth:`McapRecorder.log_message`)
so a stuck writer cannot grow the queue until the process runs out of memory.
"""

_DROP_WARNING_INTERVAL = 10.0  # seconds between 'queue full' warnings, so drops do not spam the log

QueuedMessage = tuple[str, Any, Callable[[Any, int], bytes | None] | None, int]
"""A queued entry: ``(topic, payload, encode_or_None, timestamp_ns)``.

``payload`` is already-serialized ``bytes`` when ``encode`` is ``None``; otherwise
``encode(payload, timestamp_ns)`` runs on the background writer to produce the bytes.
"""


class TopicSchema(NamedTuple):
    """Everything the writer needs to register a topic's channel and schema."""
    schema_name: str
    schema: bytes
    schema_encoding: str  # e.g. 'ros2msg' or 'jsonschema'
    message_encoding: str  # e.g. 'cdr' or 'json'


class RecordingInfo(NamedTuple):
    """A snapshot of one recording's on-disk facts, gathered off the event loop."""
    path: Path
    mtime: float
    size: int
    is_live: bool
    indexed: bool  # has a summary index; always False for the live file (not indexed until stopped)


class _DiskStats(NamedTuple):
    """Directory statistics for the developer panel, collected off the event loop."""
    current_file_size: int
    total_size: int
    file_count: int


class RecordingSource(Protocol):
    """A topic data source whose lifetime is bound to the recorder's recording state.

    Sources are activated when recording starts and deactivated when it stops, so
    event subscriptions and timers only run while a recording is actually open.
    """

    def start(self) -> None: ...
    def stop(self) -> None: ...


class McapRecorder:
    """Records sensor data to MCAP files for replay and analysis in Foxglove Studio.

    Supports automatic file rotation by size and disk budget enforcement.

    Messages are enqueued from the event loop (cheap, non-blocking) and written
    to disk by a single background consumer via ``rosys.run.io_bound`` so that
    encoding, ZSTD compression and file I/O never block the loop. Encoding runs on
    the writer too: sources enqueue the raw payload plus an ``encode`` callable,
    so no JSON/JPEG work happens on the loop. The log time is captured at enqueue,
    but encoding is deferred, so payloads are expected to be immutable value
    snapshots (as RoSys sensor events emit); a payload mutated after being enqueued
    would encode its later state.

    Two locks keep the loop responsive. ``_lock`` serializes all writer access so the
    synchronous drain in ``stop()`` cannot race the background consumer; it may be held
    for the duration of a multi-second write. ``_queue_lock`` guards only queue mutation
    (enqueue, cap-drop, swap) and is held for microseconds, so the event loop never
    blocks behind a write when appending a message. Lock order is ``_lock`` outer,
    ``_queue_lock`` inner; the loop takes ``_queue_lock`` alone, so there is no deadlock.

    The recorder is encoding-agnostic: a topic carries a :class:`TopicSchema`
    (schema name/bytes plus schema- and message-encoding). ``log_message`` takes
    either already-serialized bytes or a payload plus an ``encode`` callback.
    Conversion from application data types lives entirely in ``converters.py`` /
    ``foxglove.py``. Topics are fed by opaque :class:`RecordingSource` objects that
    are activated on ``start()`` and deactivated on ``stop()``.
    """

    def __init__(
        self,
        *,
        output_dir: Path | str = '~/.rosys/mcap',
        max_file_size_mb: float = 100,
        max_total_size_mb: float = 1000,
        chunk_size: int = 1_048_576,
        flush_interval: float = 1.0,
        profile: str = 'rosys',
        library: str = 'rosys-mcap-recorder',
        logger_name: str = 'rosys.mcap_recorder',
        auto_start: bool = True,
    ) -> None:
        self.log = logging.getLogger(logger_name)
        self.output_dir = Path(output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.max_file_size = int(max_file_size_mb * 1_048_576)
        self.max_total_size = int(max_total_size_mb * 1_048_576)
        self.chunk_size = chunk_size
        self.profile = profile
        self.library = library
        self.max_queued_messages = MAX_QUEUED_MESSAGES

        self.RECORDING_STARTED = Event[Path]()
        """a new recording file has been opened (argument: path)"""
        self.RECORDING_STOPPED = Event[Path]()
        """recording has stopped and the file is finalized (argument: path)"""

        self._declared_topics: set[str] = set()
        self._selected_topics: set[str] | None = None
        self._topic_selection: dict[str, bool] = {}  # developer-panel checkbox state, default True
        self._writer: Writer | None = None
        self._file: BinaryIO | None = None
        self._file_path: Path | None = None
        self._topics: dict[str, int] = {}
        self._schemas: dict[str, TopicSchema] = {}
        self._queue: list[QueuedMessage] = []
        self._lock = threading.Lock()  # serializes writer access; may be held for a full write
        self._queue_lock = threading.Lock()  # guards queue mutation only; held for microseconds
        self._sources: list[RecordingSource] = []
        self._message_count: int = 0
        self._dropped_message_count: int = 0
        self._last_drop_warning: float = float('-inf')
        self._is_recording: bool = False
        self._warned_topics: set[str] = set()
        self._disk_stats = _DiskStats(0, 0, 0)

        if auto_start:
            rosys.on_startup(self.start)
        rosys.on_repeat(self._flush, flush_interval)
        rosys.on_shutdown(self.stop)

    @property
    def is_recording(self) -> bool:
        return self._is_recording

    @property
    def current_file_size(self) -> int:
        if self._file_path is None:
            return 0
        return _safe_size(self._file_path)

    @property
    def total_size(self) -> int:
        return sum(_safe_size(f) for f in self.output_dir.glob('*.mcap'))

    @property
    def file_count(self) -> int:
        return len(list(self.output_dir.glob('*.mcap')))

    @property
    def topics(self) -> list[str]:
        """All declared topic names (including those whose schema is not registered yet)."""
        return sorted(self._declared_topics)

    @property
    def disabled_topics(self) -> set[str]:
        """Declared topics that the current selection drops (empty when everything is recorded)."""
        if self._selected_topics is None:
            return set()
        return self._declared_topics - self._selected_topics

    @property
    def recordings(self) -> list[Path]:
        """All recording files, newest first (by modification time, so renames keep the order)."""
        return sorted(self.output_dir.glob('*.mcap'), key=_safe_mtime, reverse=True)

    @property
    def current_recording(self) -> Path | None:
        """The file currently being written (unindexed until stopped), else None."""
        return self._file_path if self._is_recording else None

    def accepts(self, topic: str) -> bool:
        """Whether a message on ``topic`` would currently be recorded.

        Cheap, loop-safe pre-check (recording is active and the topic is not
        deselected) so callers can skip expensive encoding for topics that would
        be dropped anyway.

        :param topic: the topic name to test.
        :return: ``True`` if a message on this topic would be enqueued.
        """
        if not self._is_recording:
            return False
        return self._selected_topics is None or topic in self._selected_topics

    def scan_recordings(self) -> list[RecordingInfo]:
        """Stat every recording and check its summary index; newest first.

        Globs and stats every file and probes its summary index, which is blocking
        I/O; call via ``rosys.run.io_bound``. The live file is flagged and never
        index-probed (the writer holds it open and it has no summary index until
        stopped). Files that vanish during the scan (e.g. deleted from the
        recordings page) are skipped.

        :return: one :class:`RecordingInfo` per file, ordered newest first.
        """
        current = self.current_recording
        infos: list[RecordingInfo] = []
        for path in self.recordings:
            try:
                stat = path.stat()
            except FileNotFoundError:
                continue  # deleted concurrently while scanning
            is_live = path == current
            indexed = False if is_live else is_indexed(path)  # never read the open live file
            infos.append(RecordingInfo(path, stat.st_mtime, stat.st_size, is_live, indexed))
        return infos

    def delete_recording(self, path: Path | str) -> None:
        """Delete a recording file (only within this recorder's output directory).

        The currently-recording file is never deleted (the writer holds it open).
        """
        path = Path(path)
        if path.parent == self.output_dir and path.exists() and path != self.current_recording:
            path.unlink()
            self.log.info('deleted recording: %s', path.name)

    def delete_all_recordings(self) -> None:
        """Delete all recordings except the one currently being written."""
        for path in self.recordings:
            if path != self.current_recording:
                self.delete_recording(path)

    def rename_recording(self, path: Path | str, new_name: str) -> Path | None:
        """Rename a recording within the output directory; returns the new path or None.

        The currently-recording file cannot be renamed (the writer holds it open).
        ``new_name`` is reduced to a bare filename and given a ``.mcap`` suffix;
        empty, whitespace-only or dots-only names are rejected (they would escape
        the output directory) by returning ``None``.
        """
        path = Path(path)
        if path.parent != self.output_dir or not path.exists() or path == self.current_recording:
            return None
        name = Path(new_name.strip()).name
        if not name.strip('.'):  # empty, whitespace-only, or only dots ('.', '..', '...')
            return None
        target = self.output_dir / name
        if target.suffix != '.mcap':
            target = target.with_suffix('.mcap')
        if target == path or target.exists():
            return None
        path.rename(target)
        self.log.info('renamed recording: %s -> %s', path.name, target.name)
        return target

    def unindexed_recordings(self) -> list[Path]:
        """Finished recordings without a summary index (e.g. left by a crash).

        Opens and probes every finished file, which is blocking I/O; call via
        ``rosys.run.io_bound``.
        """
        return [path for path in self.recordings if path != self.current_recording and not is_indexed(path)]

    async def reindex_unindexed(self) -> None:
        """Rebuild the index of every unindexed recording on a background thread."""
        unindexed = await rosys.run.io_bound(self.unindexed_recordings)  # per-file open + index probe: off the loop
        for path in unindexed or []:
            self.log.warning('reindexing unindexed recording: %s', path.name)
            recovered = await rosys.run.io_bound(reindex, path)
            if recovered is not None:
                self.log.info('reindexed %s (%d messages recovered)', path.name, recovered)

    def add_topic(self, topic: str, schema: TopicSchema) -> None:
        """Register a topic with its schema and encoding.

        Can be called before or after start(). The channel is registered with
        the writer lazily on first message (or eagerly when a new file opens),
        so this method never touches the writer and is safe to call from the
        event loop while the background consumer is writing.
        """
        self._declared_topics.add(topic)
        self._schemas[topic] = schema

    def declare_topic(self, topic: str) -> None:
        """Announce a topic whose schema will only be registered on its first message.

        Auto-dispatched converters cannot know their schema before a payload arrives,
        but declaring the name up front makes the topic visible in ``topics`` — so
        selections computed before the first message (e.g. "everything but camera
        images") still cover it.
        """
        self._declared_topics.add(topic)

    def add_source(self, source: RecordingSource) -> None:
        """Register a data source whose lifetime is bound to the recording state.

        Activated immediately if recording is already running, otherwise on the
        next ``start()``.
        """
        self._sources.append(source)
        if self._is_recording:
            source.start()

    def start(self, topics: Collection[str] | None = None) -> None:
        """Start a new recording.

        :param topics: record only these topics; all others are dropped (default: record
            every registered topic). A selected topic that is registered only after the
            recording started is picked up as soon as it exists. The selection lasts for
            this recording; the next ``start()`` records everything again unless a new
            selection is passed.
        """
        if self._is_recording:
            return
        self._selected_topics = set(topics) if topics is not None else None
        self._message_count = 0
        # Budget enforcement and opening the file run on the loop here because start must
        # synchronously establish the recording before returning (callers read
        # current_recording and receive RECORDING_STARTED immediately). start is a rare
        # user/lifecycle action; the hot periodic-flush path enforces the budget off the
        # loop in the io_bound writer (see _write_batch -> _write_messages -> _rotate).
        self._enforce_disk_budget()
        with self._lock:
            self._open_new_file()
        assert self._file_path is not None
        self._is_recording = True
        for source in self._sources:
            source.start()
        self.log.info('started MCAP recording: %s', self._file_path)
        self.RECORDING_STARTED.emit(self._file_path)

    def stop(self) -> None:
        if not self._is_recording:
            return
        self._is_recording = False
        for source in self._sources:
            source.stop()
        # Drain and finalize under the lock so this synchronous stop cannot race the
        # background writer. The final drain may rotate, so read the finalized path only
        # after writing (finding: emit the actually-finalized file). Acquiring the lock
        # may briefly block the loop if a flush is mid-write, but that window is bounded
        # (one batch) and stop is rare.
        with self._lock:
            with self._queue_lock:  # brief: only the swap, not the write
                batch, self._queue = self._queue, []
            self._write_messages(batch)
            file_path = self._file_path
            self._close_file()
        if file_path is not None and self._message_count == 0:
            file_path.unlink(missing_ok=True)  # discard empty recordings (e.g. from rapid toggling)
            self.log.info('discarded empty recording: %s', file_path.name)
        else:
            self.log.info('stopped MCAP recording (%d messages total)', self._message_count)
            if file_path is not None:
                self.RECORDING_STOPPED.emit(file_path)

    def log_message(self, topic: str, data: Any, *,
                    encode: Callable[[Any, int], bytes | None] | None = None,
                    timestamp_ns: int | None = None) -> None:
        """Enqueue a message for the background writer.

        :param topic: the (registered) topic to write to; unknown topics are dropped
            with a one-time warning.
        :param data: already-serialized ``bytes`` when ``encode`` is ``None``, otherwise
            the raw payload passed to ``encode`` on the writer thread.
        :param encode: optional ``(payload, timestamp_ns) -> bytes | None`` run off the
            event loop by the background writer; returning ``None`` drops the message.
        :param timestamp_ns: log time in nanoseconds (default: current ``rosys.time()``).
        """
        if not self.accepts(topic):
            return
        if topic not in self._schemas:
            if topic not in self._warned_topics:
                self.log.warning('unknown topic: %s', topic)
                self._warned_topics.add(topic)
            return
        if timestamp_ns is None:
            timestamp_ns = int(rosys.time() * NANOSECONDS_PER_SECOND)
        with self._queue_lock:  # microsecond hold; never blocks behind a write
            self._queue.append((topic, data, encode, timestamp_ns))
            self._enforce_queue_cap()

    def _enforce_queue_cap(self) -> None:
        """Drop the oldest queued messages when the disk cannot keep up. Caller holds ``_queue_lock``.

        Bounds memory if the writer stalls; the drop is logged at most once per
        ``_DROP_WARNING_INTERVAL`` so a persistent stall does not flood the log.
        """
        overflow = len(self._queue) - self.max_queued_messages
        if overflow <= 0:
            return
        del self._queue[:overflow]
        self._dropped_message_count += overflow
        now = rosys.time()
        if now - self._last_drop_warning >= _DROP_WARNING_INTERVAL:
            self.log.warning('recording queue full at %d messages; dropping oldest — disk cannot keep up '
                             '(%d dropped so far)', self.max_queued_messages, self._dropped_message_count)
            self._last_drop_warning = now

    async def _flush(self) -> None:
        if not self._is_recording or not self._queue:
            return
        await rosys.run.io_bound(self._write_batch)

    def _write_batch(self) -> None:
        """Drain the queue and write it. Runs on the background writer thread."""
        with self._lock:
            with self._queue_lock:  # brief: only the swap, not the write
                batch, self._queue = self._queue, []
            self._write_messages(batch)

    def _write_messages(self, batch: list[QueuedMessage]) -> None:
        """Encode and write a batch of messages. Caller must hold ``_lock``."""
        if self._writer is None:
            return
        for topic, payload, encode, timestamp_ns in batch:
            schema = self._schemas.get(topic)
            if schema is None:
                continue
            data = encode(payload, timestamp_ns) if encode is not None else payload
            if data is None:
                continue  # converter chose to skip this value
            if topic not in self._topics:
                self._register_topic(topic, schema)
            self._writer.add_message(
                channel_id=self._topics[topic],
                log_time=timestamp_ns,
                data=data,
                publish_time=timestamp_ns,
            )
            self._message_count += 1
            assert self._file is not None
            if self._file.tell() >= self.max_file_size:
                self._rotate()
                if self._writer is None:
                    return

    def _register_topic(self, topic: str, schema: TopicSchema) -> None:
        assert self._writer is not None
        schema_id = self._writer.register_schema(
            name=schema.schema_name,
            encoding=schema.schema_encoding,
            data=schema.schema,
        )
        self._topics[topic] = self._writer.register_channel(
            schema_id=schema_id,
            topic=topic,
            message_encoding=schema.message_encoding,
        )

    def _open_new_file(self) -> None:
        timestamp = datetime.now(tz=UTC).strftime('%Y%m%d_%H%M%S_%f')  # microseconds -> unique per file
        self._file_path = self.output_dir / f'{timestamp}.mcap'
        self._file = open(self._file_path, 'wb')  # pylint: disable=consider-using-with
        self._writer = Writer(self._file, compression=CompressionType.ZSTD, chunk_size=self.chunk_size)
        self._writer.start(profile=self.profile, library=self.library)
        self._topics.clear()
        for topic, schema in self._schemas.items():
            if self._selected_topics is None or topic in self._selected_topics:
                self._register_topic(topic, schema)

    def _close_file(self) -> None:
        try:
            if self._writer is not None:
                self._writer.finish()
        finally:
            self._writer = None
            if self._file is not None:
                self._file.close()
                self._file = None
            self._topics.clear()

    def _rotate(self) -> None:
        """Close the current file and start a fresh one. Caller must hold ``_lock``."""
        assert self._file is not None
        self.log.info('rotating MCAP file (%.1f MB)', self._file.tell() / 1_048_576)
        self._close_file()
        self._enforce_disk_budget()
        self._open_new_file()

    def _enforce_disk_budget(self) -> None:
        file_stats: list[tuple[Path, int, float]] = []
        for path in self.output_dir.glob('*.mcap'):
            try:
                stat = path.stat()
            except FileNotFoundError:
                continue  # vanished concurrently (e.g. deleted from the recordings page)
            file_stats.append((path, stat.st_size, stat.st_mtime))
        file_stats.sort(key=lambda item: item[2])  # oldest first
        total = sum(size for _, size, _ in file_stats)
        while total > self.max_total_size and file_stats:
            oldest, size, _ = file_stats.pop(0)
            oldest.unlink(missing_ok=True)
            total -= size
            self.log.info('deleted old recording: %s (freed %.1f MB)', oldest.name, size / 1_048_576)

    def developer_ui(self) -> None:
        """Developer panel: auto-refreshing stats, start/stop buttons, and a topic selection.

        Only the stats grid is rebuilt on the timer; the buttons are built once and
        toggle their visibility reactively, so they never flicker. The timer refreshes
        cached directory stats collected off the event loop (globbing and statting the
        directory would otherwise block the loop every second) and is bound to the
        client, so it is cleaned up on disconnect (no global event subscriptions).
        The topic checkboxes live in a collapsed expansion to keep the panel compact;
        the selection applies to the next start (a running recording is unaffected).
        """
        with ui.column():
            ui.label('MCAP Recording').classes('text-center text-bold')
            self._stats_ui()
            ui.timer(1.0, self._refresh_stats)
            with ui.row().classes('items-center'):
                ui.button(icon='fiber_manual_record', color='red', on_click=self._start_with_selection) \
                    .tooltip('Start recording') \
                    .bind_visibility_from(self, 'is_recording', backward=lambda recording: not recording)
                ui.button(icon='stop', color='red', on_click=self.stop).tooltip('Stop recording') \
                    .bind_visibility_from(self, 'is_recording')
                ui.link('Recordings', PAGE_PATH)
            # topics can still be declared after page build, so the list is rebuilt
            # every time the expansion is opened rather than once at page build
            expansion = ui.expansion('Topics').props('dense').classes('w-full')
            with expansion:
                self._topic_selection_ui()
            expansion.on_value_change(self._topic_selection_ui.refresh)

    def _start_with_selection(self) -> None:
        """Start recording the topics selected in the developer panel (default: all)."""
        selected = [topic for topic in self.topics if self._topic_selection.get(topic, True)]
        if not selected:
            rosys.notify('Select at least one topic to record', type='negative')
            return
        self.start(topics=None if len(selected) == len(self.topics) else selected)

    @ui.refreshable_method
    def _topic_selection_ui(self) -> None:
        def set_all(value: bool) -> None:
            for topic in self.topics:
                self._topic_selection[topic] = value
            self._topic_selection_ui.refresh()

        with ui.row().classes('gap-2'):
            ui.button('all', on_click=lambda: set_all(True)).props('flat dense')
            ui.button('none', on_click=lambda: set_all(False)).props('flat dense')
        with ui.column().classes('gap-0'):
            for topic in sorted(self.topics):
                ui.checkbox(topic, value=self._topic_selection.get(topic, True),
                            on_change=lambda event, topic=topic: self._topic_selection.__setitem__(topic, event.value)) \
                    .props('dense size=xs')

    async def _refresh_stats(self) -> None:
        """Refresh the cached directory stats off the event loop, then rebuild the stats grid."""
        stats = await rosys.run.io_bound(self._collect_disk_stats)
        if stats is not None:
            self._disk_stats = stats
        self._stats_ui.refresh()

    def _collect_disk_stats(self) -> _DiskStats:
        """Stat the output directory (blocking I/O; run off the loop)."""
        return _DiskStats(self.current_file_size, self.total_size, self.file_count)

    @ui.refreshable_method
    def _stats_ui(self) -> None:
        mb = 1_048_576
        stats = self._disk_stats
        with ui.grid(columns='auto auto').classes('gap-y-1'):
            ui.label('Messages:')
            ui.label(str(self._message_count))
            ui.label('File:')
            ui.label(self._file_path.name if self._file_path else '-')
            ui.label('File size:')
            ui.label(f'{stats.current_file_size / mb:.1f} MB')
            ui.label('Total:')
            ui.label(f'{stats.total_size / mb:.1f} / {self.max_total_size / mb:.0f} MB')
            ui.label('Files:')
            ui.label(str(stats.file_count))


def _safe_mtime(path: Path) -> float:
    """Modification time of ``path``, or 0 if it vanished (e.g. deleted concurrently)."""
    try:
        return path.stat().st_mtime
    except FileNotFoundError:
        return 0.0


def _safe_size(path: Path) -> int:
    """Size of ``path`` in bytes, or 0 if it vanished (e.g. deleted concurrently)."""
    try:
        return path.stat().st_size
    except FileNotFoundError:
        return 0
