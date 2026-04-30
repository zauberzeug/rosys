import json
import tempfile
from pathlib import Path

import pytest
from mcap.reader import make_reader

from rosys.recording import McapLogger


@pytest.fixture
def mcap_dir():
    with tempfile.TemporaryDirectory(prefix='rosys-mcap-') as tmp:
        yield Path(tmp)


def test_write_and_read_messages(mcap_dir: Path) -> None:
    logger = McapLogger(output_dir=mcap_dir, auto_start=False)
    logger.add_topic('/test', schema_name='TestMessage', schema={
        'type': 'object',
        'properties': {'value': {'type': 'number'}},
    })
    logger.start()

    for i in range(5):
        logger.log_message('/test', {'value': i}, timestamp_ns=i * 1_000_000_000)

    logger.stop()

    assert logger._message_count == 5
    files = list(mcap_dir.glob('*.mcap'))
    assert len(files) == 1

    with open(files[0], 'rb') as f:
        reader = make_reader(f)
        messages = [(json.loads(msg.data), msg.log_time) for _, _, msg in reader.iter_messages()]
    assert len(messages) == 5
    assert messages[0][0] == {'value': 0}
    assert messages[4][0] == {'value': 4}
    assert messages[2][1] == 2_000_000_000


def test_file_rotation(mcap_dir: Path) -> None:
    logger = McapLogger(output_dir=mcap_dir, max_file_size_mb=0.001, auto_start=False)
    logger.add_topic('/test', schema_name='TestMessage', schema={
        'type': 'object',
        'properties': {'payload': {'type': 'string'}},
    })
    logger.start()

    large_payload = 'x' * 500
    for i in range(20):
        logger.log_message('/test', {'payload': large_payload}, timestamp_ns=i * 1_000_000_000)

    logger.stop()

    files = list(mcap_dir.glob('*.mcap'))
    assert len(files) >= 2, f'Expected rotation, got {len(files)} file(s)'


def test_disk_budget_enforcement(mcap_dir: Path) -> None:
    logger = McapLogger(
        output_dir=mcap_dir,
        max_file_size_mb=0.001,
        max_total_size_mb=0.003,
        auto_start=False,
    )
    logger.add_topic('/test', schema_name='TestMessage', schema={
        'type': 'object',
        'properties': {'payload': {'type': 'string'}},
    })
    logger.start()

    large_payload = 'x' * 500
    for i in range(100):
        logger.log_message('/test', {'payload': large_payload}, timestamp_ns=i * 1_000_000_000)

    logger.stop()

    total = sum(f.stat().st_size for f in mcap_dir.glob('*.mcap'))
    max_allowed = 0.003 * 1_048_576 + 0.001 * 1_048_576  # budget + one active file
    assert total <= max_allowed, f'Total {total} exceeds budget+active {max_allowed}'


def test_unknown_topic_ignored(mcap_dir: Path) -> None:
    logger = McapLogger(output_dir=mcap_dir, auto_start=False)
    logger.start()

    logger.log_message('/nonexistent', {'value': 1}, timestamp_ns=0)

    logger.stop()
    assert logger._message_count == 0


def test_add_topic_while_recording(mcap_dir: Path) -> None:
    logger = McapLogger(output_dir=mcap_dir, auto_start=False)
    logger.start()

    logger.add_topic('/late', schema_name='LateMessage', schema={
        'type': 'object',
        'properties': {'value': {'type': 'number'}},
    })
    logger.log_message('/late', {'value': 42}, timestamp_ns=0)
    logger.stop()

    assert logger._message_count == 1
    files = list(mcap_dir.glob('*.mcap'))
    with open(files[0], 'rb') as f:
        reader = make_reader(f)
        messages = [json.loads(msg.data) for _, _, msg in reader.iter_messages()]
    assert messages == [{'value': 42}]


def test_compression_options(mcap_dir: Path) -> None:
    for comp in ('zstd', 'lz4', 'none'):
        subdir = mcap_dir / comp
        subdir.mkdir()
        logger = McapLogger(output_dir=subdir, compression=comp, auto_start=False)
        logger.add_topic('/test', schema_name='Test', schema={
            'type': 'object',
            'properties': {'v': {'type': 'number'}},
        })
        logger.start()
        logger.log_message('/test', {'v': 1}, timestamp_ns=0)
        logger.stop()

        files = list(subdir.glob('*.mcap'))
        assert len(files) == 1, f'compression={comp} failed'
        with open(files[0], 'rb') as f:
            reader = make_reader(f)
            messages = [json.loads(msg.data) for _, _, msg in reader.iter_messages()]
        assert messages == [{'v': 1}]


def test_stop_without_start(mcap_dir: Path) -> None:
    logger = McapLogger(output_dir=mcap_dir, auto_start=False)
    logger.stop()
    assert not logger.is_recording
    assert list(mcap_dir.glob('*.mcap')) == []


def test_double_start(mcap_dir: Path) -> None:
    logger = McapLogger(output_dir=mcap_dir, auto_start=False)
    logger.add_topic('/test', schema_name='Test', schema={
        'type': 'object',
        'properties': {'v': {'type': 'number'}},
    })
    logger.start()
    first_file = logger._file_path
    logger.start()
    assert logger._file_path == first_file
    logger.stop()
