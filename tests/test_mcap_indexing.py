import json
from pathlib import Path

import pytest
from mcap.writer import CompressionType, Writer

from rosys.analysis.recording import McapRecorder, TopicSchema, is_indexed, reindex
from rosys.analysis.recording.indexing import _reindex_temp_path

NS = 1_000_000_000


def _write_unindexed(path: Path, messages: int = 2000) -> None:
    """Write a chunked MCAP file but never call finish() — like a crashed process."""
    with open(path, 'wb') as file:
        writer = Writer(file, compression=CompressionType.ZSTD, chunk_size=4096)
        writer.start(profile='', library='test')
        schema_id = writer.register_schema(name='T', encoding='jsonschema', data=b'{}')
        channel_id = writer.register_channel(schema_id=schema_id, topic='/t', message_encoding='json')
        for i in range(messages):
            writer.add_message(channel_id=channel_id, log_time=i * NS,
                               data=json.dumps({'v': i, 'pad': 'x' * 200}).encode(), publish_time=i * NS)
        file.flush()  # no finish() -> no summary index


async def test_finished_recording_is_indexed(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/t', TopicSchema('T', b'{}', 'jsonschema', 'json'))
    recorder.start()
    recorder.log_message('/t', b'{"v": 1}', timestamp_ns=NS)
    await recorder.stop()

    assert is_indexed(recorder.recordings[0])
    assert recorder.unindexed_recordings() == []


def test_reindex_recovers_and_indexes(mcap_dir: Path) -> None:
    path = mcap_dir / 'crash.mcap'
    _write_unindexed(path)
    assert not is_indexed(path)

    recovered = reindex(path)

    assert recovered > 0
    assert is_indexed(path)


async def test_reindex_unindexed_via_recorder(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    _write_unindexed(mcap_dir / 'crash.mcap')
    assert len(recorder.unindexed_recordings()) == 1

    await recorder.reindex_unindexed()

    assert recorder.unindexed_recordings() == []


def test_reindex_recovers_readable_prefix_of_truncated_file(mcap_dir: Path) -> None:
    """A recording whose tail is cut off mid-chunk still reindexes its readable prefix."""
    path = mcap_dir / 'crash.mcap'
    _write_unindexed(path)
    path.write_bytes(path.read_bytes()[:-100])  # drop the last bytes, as a crash mid-write would
    assert not is_indexed(path)

    recovered = reindex(path)

    assert recovered > 0
    assert is_indexed(path)


def test_reindex_write_failure_keeps_original_and_leaves_no_temp(mcap_dir: Path,
                                                                 monkeypatch: pytest.MonkeyPatch) -> None:
    """A write failure mid-rebuild re-raises and never replaces the complete original with a truncation."""
    path = mcap_dir / 'crash.mcap'
    _write_unindexed(path)
    original = path.read_bytes()

    def fail(*args: object, **kwargs: object) -> None:
        raise OSError('No space left on device')

    monkeypatch.setattr(Writer, 'add_message', fail)

    with pytest.raises(OSError, match='No space left on device'):
        reindex(path)

    assert path.read_bytes() == original  # the only complete copy is untouched
    assert list(mcap_dir.glob('*.reindex*')) == []  # no half-written temp left behind


def test_reindex_temp_paths_are_unique(mcap_dir: Path) -> None:
    """Two reindex runs of the same recording get distinct temp files, so they cannot collide."""
    path = mcap_dir / 'crash.mcap'

    first, second = _reindex_temp_path(path), _reindex_temp_path(path)

    assert first != second  # a fixed name would let overlapping runs overwrite each other
    assert first.parent == path.parent == second.parent  # same directory keeps replace() atomic
    assert first.name.startswith('crash.mcap.reindex-')  # startup cleanup globs '*.mcap.reindex*'
    assert second.name.startswith('crash.mcap.reindex-')
    assert first.suffix != '.mcap'  # the download endpoint must never serve a temp as a recording


async def test_current_recording_excluded_from_unindexed(mcap_dir: Path) -> None:
    recorder = McapRecorder(output_dir=mcap_dir, auto_start=False)
    recorder.add_topic('/t', TopicSchema('T', b'{}', 'jsonschema', 'json'))
    recorder.start()  # active file has no index yet, but must not count as a reindex orphan
    assert recorder.unindexed_recordings() == []
    await recorder.stop()
