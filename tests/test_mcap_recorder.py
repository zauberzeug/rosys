import json
import os
import tempfile
from pathlib import Path

import pytest
from mcap.reader import make_reader

from rosys.analysis.recording import McapRecorder, TopicSchema

NS = 1_000_000_000


def _schema(name: str = 'Test', properties: dict | None = None) -> TopicSchema:
    body = {'type': 'object', 'properties': properties or {'value': {'type': 'number'}}}
    return TopicSchema(name, json.dumps(body).encode(), 'jsonschema', 'json')


def _json(data: dict) -> bytes:
    return json.dumps(data).encode()


@pytest.fixture
def mcap_dir():
    with tempfile.TemporaryDirectory(prefix='rosys-mcap-') as tmp:
        yield Path(tmp)


def _values(path: Path) -> list:
    """Read the ``value`` field of every JSON message in a recording, in file order.

    :param path: the MCAP file to read.
    :return: the recorded ``value`` of each message.
    """
    with open(path, 'rb') as f:
        return [json.loads(msg.data)['value'] for _, _, msg in make_reader(f).iter_messages()]


def test_write_and_read_messages(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()

    for i in range(5):
        recorder.log_message('/test', _json({'value': i}), timestamp_ns=i * NS)

    recorder.stop()

    assert recorder._message_count == 5
    files = list(mcap_dir.glob('*.mcap'))
    assert len(files) == 1

    with open(files[0], 'rb') as f:
        reader = make_reader(f)
        messages = [(json.loads(msg.data), msg.log_time) for _, _, msg in reader.iter_messages()]
    assert len(messages) == 5
    assert messages[0][0] == {'value': 0}
    assert messages[4][0] == {'value': 4}
    assert messages[2][1] == 2 * NS


def test_file_rotation(mcap_dir: Path) -> None:
    # Rotation triggers on actual on-disk (compressed) size, so use incompressible
    # payloads and a small chunk size to make the writer flush bytes frequently.
    recorder = McapRecorder(output_dir=mcap_dir, max_file_size_mb=0.05, chunk_size=4096, auto_start=False)
    recorder.add_topic('/test', _schema('Test', {'payload': {'type': 'string'}}))
    recorder.start()

    for i in range(200):
        recorder.log_message('/test', _json({'payload': os.urandom(1024).hex()}), timestamp_ns=i * NS)

    recorder.stop()

    files = list(mcap_dir.glob('*.mcap'))
    assert len(files) >= 2, f'Expected rotation, got {len(files)} file(s)'


def test_disk_budget_enforcement(mcap_dir: Path) -> None:
    recorder = McapRecorder(
        output_dir=mcap_dir,
        max_file_size_mb=0.05,
        max_total_size_mb=0.15,
        chunk_size=4096,
        auto_start=False,
    )
    recorder.add_topic('/test', _schema('Test', {'payload': {'type': 'string'}}))
    recorder.start()

    for i in range(400):
        recorder.log_message('/test', _json({'payload': os.urandom(1024).hex()}), timestamp_ns=i * NS)

    recorder.stop()

    total = sum(f.stat().st_size for f in mcap_dir.glob('*.mcap'))
    max_allowed = (0.15 + 0.05) * 1_048_576  # budget + one active (not-yet-rotated) file
    assert total <= max_allowed, f'Total {total} exceeds budget+active {max_allowed}'


def test_unknown_topic_ignored(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.start()

    recorder.log_message('/nonexistent', _json({'value': 1}), timestamp_ns=0)

    recorder.stop()
    assert recorder._message_count == 0


def test_add_topic_while_recording(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.start()

    recorder.add_topic('/late', _schema('LateMessage'))
    recorder.log_message('/late', _json({'value': 42}), timestamp_ns=0)
    recorder.stop()

    assert recorder._message_count == 1
    files = list(mcap_dir.glob('*.mcap'))
    with open(files[0], 'rb') as f:
        reader = make_reader(f)
        messages = [json.loads(msg.data) for _, _, msg in reader.iter_messages()]
    assert messages == [{'value': 42}]


def test_topic_selection_records_only_selected(mcap_dir: Path) -> None:
    """Starting with a topic selection drops every message on unselected topics."""
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/wanted', _schema('Wanted'))
    recorder.add_topic('/unwanted', _schema('Unwanted'))
    recorder.start(topics=['/wanted'])

    recorder.log_message('/wanted', _json({'value': 1}), timestamp_ns=0)
    recorder.log_message('/unwanted', _json({'value': 2}), timestamp_ns=1)
    recorder.log_message('/wanted', _json({'value': 3}), timestamp_ns=2)
    recorder.stop()

    files = list(mcap_dir.glob('*.mcap'))
    with open(files[0], 'rb') as f:
        reader = make_reader(f)
        messages = [(channel.topic, json.loads(msg.data)) for _, channel, msg in reader.iter_messages()]
    assert messages == [('/wanted', {'value': 1}), ('/wanted', {'value': 3})]


def test_topic_selection_includes_late_registration(mcap_dir: Path) -> None:
    """A selected topic that is registered only after start is recorded once it exists."""
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.start(topics=['/late'])

    recorder.add_topic('/late', _schema('LateMessage'))
    recorder.log_message('/late', _json({'value': 42}), timestamp_ns=0)
    recorder.stop()

    assert recorder._message_count == 1


def test_declared_topic_selectable_before_schema_registration(mcap_dir: Path) -> None:
    """A declared (schema-pending) topic is excludable by a selection computed from ``topics``.

    Auto-dispatched topics register their schema only on the first message, which
    arrives after the recording started; the selection must still be able to drop them.
    """
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/pose', _schema('Pose'))
    recorder.declare_topic('/camera/main/image')
    assert recorder.topics == ['/camera/main/image', '/pose']

    recorder.start(topics=[topic for topic in recorder.topics if not topic.endswith('/image')])
    recorder.add_topic('/camera/main/image', _schema('Image'))  # auto-dispatch on first message
    recorder.log_message('/camera/main/image', _json({'value': 1}), timestamp_ns=0)
    recorder.log_message('/pose', _json({'value': 2}), timestamp_ns=1)
    recorder.stop()

    assert recorder._message_count == 1


def test_rotation_registers_only_selected_channels(mcap_dir: Path) -> None:
    """Rotated files do not declare channels for topics the selection drops."""
    recorder = McapRecorder(output_dir=mcap_dir, max_file_size_mb=0.05, chunk_size=4096, auto_start=False)
    recorder.add_topic('/wanted', _schema('Wanted', {'payload': {'type': 'string'}}))
    recorder.add_topic('/unwanted', _schema('Unwanted'))
    recorder.start(topics=['/wanted'])

    for i in range(200):  # incompressible payloads to force at least one rotation
        recorder.log_message('/wanted', _json({'payload': os.urandom(1024).hex()}), timestamp_ns=i * NS)
    recorder.stop()

    files = list(mcap_dir.glob('*.mcap'))
    assert len(files) >= 2, f'Expected rotation, got {len(files)} file(s)'
    for file in files:
        with open(file, 'rb') as f:
            channels = make_reader(f).get_summary().channels.values()
        assert [channel.topic for channel in channels] == ['/wanted']


def test_disabled_topics_reflects_selection(mcap_dir: Path) -> None:
    """``disabled_topics`` lists what the current selection drops and resets with it."""
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/a', _schema('A'))
    recorder.add_topic('/b', _schema('B'))
    assert recorder.disabled_topics == set()

    recorder.start(topics=['/a'])
    assert recorder.disabled_topics == {'/b'}
    recorder.stop()

    recorder.start()
    assert recorder.disabled_topics == set()
    recorder.stop()


def test_topic_selection_resets_on_next_start(mcap_dir: Path) -> None:
    """Without a new selection, the next recording captures every topic again."""
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/a', _schema('A'))
    recorder.add_topic('/b', _schema('B'))
    recorder.start(topics=['/a'])
    recorder.log_message('/b', _json({'value': 1}), timestamp_ns=0)
    recorder.stop()

    recorder.start()
    recorder.log_message('/a', _json({'value': 2}), timestamp_ns=0)
    recorder.log_message('/b', _json({'value': 3}), timestamp_ns=1)
    recorder.stop()

    assert recorder._message_count == 2


def test_stop_without_start(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.stop()
    assert not recorder.is_recording
    assert list(mcap_dir.glob('*.mcap')) == []


def test_double_start(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()
    first_file = recorder._file_path
    recorder.start()
    assert recorder._file_path == first_file
    recorder.stop()


async def test_background_flush_writes_messages(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()

    for i in range(5):
        recorder.log_message('/test', _json({'value': i}), timestamp_ns=i * NS)

    await recorder._flush()  # drain off the loop via rosys.run.io_bound
    assert recorder._message_count == 5
    assert not recorder._queue

    recorder.stop()
    files = list(mcap_dir.glob('*.mcap'))
    with open(files[0], 'rb') as f:
        reader = make_reader(f)
        values = [json.loads(msg.data)['value'] for _, _, msg in reader.iter_messages()]
    assert values == [0, 1, 2, 3, 4]


def test_recording_events_fire(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    started: list[Path] = []
    stopped: list[Path] = []
    recorder.RECORDING_STARTED.subscribe(started.append)
    recorder.RECORDING_STOPPED.subscribe(stopped.append)

    recorder.start()
    recorder.log_message('/test', _json({'value': 1}), timestamp_ns=NS)
    recorder.stop()

    assert len(started) == 1
    assert started[0].suffix == '.mcap'
    assert stopped == started  # same finalized file path


def test_no_stopped_event_for_discarded_empty_recording(mcap_dir: Path) -> None:
    """RECORDING_STOPPED promises a finalized file, so a discarded empty recording must not emit it."""
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    stopped: list[Path] = []
    recorder.RECORDING_STOPPED.subscribe(stopped.append)

    recorder.start()
    recorder.stop()  # nothing recorded -> file deleted, no event

    assert stopped == []


def test_recordings_listing_and_delete(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()
    recorder.log_message('/test', _json({'value': 1}), timestamp_ns=NS)
    recorder.stop()

    assert len(recorder.recordings) == 1
    path = recorder.recordings[0]
    recorder.delete_recording(path)
    assert recorder.recordings == []


def test_scan_recordings_reports_live_and_index_state(mcap_dir: Path) -> None:
    """The scan flags the file being written as live/unindexed and finished files as indexed."""
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()
    recorder.log_message('/test', _json({'value': 1}), timestamp_ns=NS)

    live_infos = recorder.scan_recordings()
    assert len(live_infos) == 1
    assert live_infos[0].is_live
    assert not live_infos[0].indexed

    recorder.stop()
    infos = recorder.scan_recordings()
    assert len(infos) == 1
    assert not infos[0].is_live
    assert infos[0].indexed
    assert infos[0].size > 0
    assert infos[0].path == recorder.recordings[0]


def test_delete_all_recordings(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    for i in range(3):
        recorder.start()
        recorder.log_message('/test', _json({'value': i}), timestamp_ns=i * NS)
        recorder.stop()
    assert recorder.recordings  # some were created

    recorder.delete_all_recordings()
    assert recorder.recordings == []


def test_delete_active_recording_refused(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()
    active = recorder.current_recording
    assert active is not None
    recorder.delete_recording(active)  # the open file must not be deleted
    assert active.exists()
    recorder.stop()


def test_delete_all_keeps_active_recording(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()
    active = recorder.current_recording
    recorder.delete_all_recordings()  # must not delete the file being written
    assert active is not None and active.exists()
    recorder.stop()


def test_empty_recording_discarded(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()
    recorder.stop()  # nothing recorded -> file discarded (e.g. rapid toggling)
    assert list(mcap_dir.glob('*.mcap')) == []


def test_nonempty_recording_kept(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()
    recorder.log_message('/test', _json({'value': 1}), timestamp_ns=NS)
    recorder.stop()
    assert len(list(mcap_dir.glob('*.mcap'))) == 1


def test_rename_recording(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()
    recorder.log_message('/test', _json({'value': 1}), timestamp_ns=NS)
    recorder.stop()
    path = recorder.recordings[0]

    new_path = recorder.rename_recording(path, 'my run')

    assert new_path is not None
    assert new_path.name == 'my run.mcap'
    assert recorder.recordings == [new_path]
    assert not path.exists()


def test_rename_active_recording_refused(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.start()
    active = recorder.current_recording
    assert recorder.rename_recording(active, 'renamed') is None  # the open file cannot be renamed
    recorder.stop()


def test_mixed_encodings_in_one_file(mcap_dir: Path) -> None:
    # The recorder is encoding-agnostic: different topics may use different encodings.
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/json', TopicSchema('J', b'{}', 'jsonschema', 'json'))
    recorder.add_topic('/raw', TopicSchema('R', b'raw', 'ros2msg', 'cdr'))
    recorder.start()
    recorder.log_message('/json', b'{"a": 1}', timestamp_ns=NS)
    recorder.log_message('/raw', b'\x01\x02\x03', timestamp_ns=NS)
    recorder.stop()

    encodings = {}
    file = next(iter(mcap_dir.glob('*.mcap')))
    with open(file, 'rb') as f:
        reader = make_reader(f)
        for _, channel, message in reader.iter_messages():
            encodings[channel.topic] = (channel.message_encoding, bytes(message.data))
    assert encodings['/json'] == ('json', b'{"a": 1}')
    assert encodings['/raw'] == ('cdr', b'\x01\x02\x03')


def test_stop_writes_all_enqueued_messages_exactly_once(mcap_dir: Path) -> None:
    """stop() drains every message still queued (never flushed), writing each exactly once."""
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()
    for i in range(20):
        recorder.log_message('/test', _json({'value': i}), timestamp_ns=i * NS)  # nothing flushes before stop
    recorder.stop()

    assert recorder._message_count == 20
    assert _values(recorder.recordings[0]) == list(range(20))


def test_stop_does_not_leak_messages_into_next_recording(mcap_dir: Path) -> None:
    """Messages queued before stop() land in their own file, never in the next recording's file."""
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()
    first = recorder.current_recording
    for i in range(5):
        recorder.log_message('/test', _json({'value': i}), timestamp_ns=i * NS)
    recorder.stop()

    recorder.start()
    second = recorder.current_recording
    recorder.log_message('/test', _json({'value': 99}), timestamp_ns=99 * NS)
    recorder.stop()

    assert first is not None and second is not None and first != second
    assert _values(first) == [0, 1, 2, 3, 4]
    assert _values(second) == [99]


async def test_flush_then_stop_writes_every_batch(mcap_dir: Path) -> None:
    """A background flush followed by stop() writes both the flushed and the remaining messages, in order."""
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()
    for i in range(5):
        recorder.log_message('/test', _json({'value': i}), timestamp_ns=i * NS)
    await recorder._flush()  # first batch drained off the loop
    for i in range(5, 10):
        recorder.log_message('/test', _json({'value': i}), timestamp_ns=i * NS)
    recorder.stop()  # second batch drained synchronously

    assert recorder._message_count == 10
    assert _values(recorder.recordings[0]) == list(range(10))


def test_queue_cap_drops_oldest_messages(mcap_dir: Path) -> None:
    """When the writer cannot keep up, the queue is capped and the oldest messages are dropped."""
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.max_queued_messages = 3
    recorder.start()
    for i in range(5):
        recorder.log_message('/test', _json({'value': i}), timestamp_ns=i * NS)  # nothing flushes

    assert len(recorder._queue) == 3  # capped
    assert recorder._dropped_message_count == 2
    recorder.stop()

    assert _values(recorder.recordings[0]) == [2, 3, 4]  # the two oldest were dropped


@pytest.mark.parametrize('bad_name', ['.', '..', '...', '/', '   '])
def test_rename_rejects_names_that_escape_output_dir(mcap_dir: Path, bad_name: str) -> None:
    """Renaming to '.', '/', whitespace or dots-only is rejected instead of writing outside the output dir."""
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()
    recorder.log_message('/test', _json({'value': 1}), timestamp_ns=NS)
    recorder.stop()
    path = recorder.recordings[0]

    assert recorder.rename_recording(path, bad_name) is None
    assert path.exists()  # the original file is untouched
    assert recorder.recordings == [path]  # nothing new appeared, nothing escaped the directory


def test_rename_rejects_collision(mcap_dir: Path) -> None:
    """Renaming onto an existing file name is rejected (returns None, both files kept)."""
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/test', _schema())
    recorder.start()
    recorder.log_message('/test', _json({'value': 1}), timestamp_ns=NS)
    recorder.stop()
    alpha = recorder.rename_recording(recorder.recordings[0], 'alpha')
    assert alpha is not None

    recorder.start()
    recorder.log_message('/test', _json({'value': 2}), timestamp_ns=NS)
    recorder.stop()
    other = next(path for path in recorder.recordings if path != alpha)

    assert recorder.rename_recording(other, 'alpha') is None  # 'alpha.mcap' already exists
    assert alpha.exists() and other.exists()


def test_start_deletes_old_recordings_over_budget(mcap_dir: Path) -> None:
    """start() enforces the disk budget on entry, deleting the oldest recordings before opening the new file."""
    for i in range(3):
        path = mcap_dir / f'old_{i}.mcap'
        path.write_bytes(os.urandom(60 * 1024))  # 60 KiB each -> 180 KiB total
        os.utime(path, (i, i))  # ascending mtimes so old_0 is the oldest
    recorder = McapRecorder(output_dir=mcap_dir, max_total_size_mb=0.12, auto_start=False)  # ~126 KiB budget

    recorder.start()  # must delete the oldest file(s) to get under budget before opening the new file
    recorder.stop()  # empty -> new file discarded

    remaining = {path.name for path in mcap_dir.glob('*.mcap')}
    assert 'old_0.mcap' not in remaining  # oldest deleted to satisfy the budget
    assert 'old_2.mcap' in remaining  # newest kept
