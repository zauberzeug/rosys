import asyncio
import json
from pathlib import Path

import pytest
from mcap.reader import make_reader
from mcap.writer import Writer

import rosys
from rosys.analysis.recording import (
    MERGE_METADATA_NAME,
    METADATA_NAME,
    McapRecorder,
    TopicSchema,
    is_auto_named,
    merge_recordings,
)
from rosys.analysis.recording.merging import merge_into_place

NS = 1_000_000_000


def _schema(properties: dict | None = None) -> TopicSchema:
    body = {'type': 'object', 'properties': properties or {'value': {'type': 'number'}}}
    return TopicSchema('Test', json.dumps(body).encode(), 'jsonschema', 'json')


def _recorder(mcap_dir: Path) -> McapRecorder:
    recorder = McapRecorder(output_dir=mcap_dir, max_file_duration=60, auto_start=False)
    recorder.add_topic('/test', _schema())
    return recorder


async def _record(recorder: McapRecorder, parts: list[list[int]], **start_arguments) -> list[Path]:
    """Record one run whose parts hold the given values, letting a rotation separate them.

    :param recorder: the recorder to record with.
    :param parts: the values of each part, in recording order.
    :param start_arguments: passed on to ``start()``.
    :return: the parts of the run, oldest first.
    """
    run = recorder.start(**start_arguments)
    for index, values in enumerate(parts):
        if index:
            rosys.set_time(rosys.time() + 61)
        for value in values:
            recorder.log_message('/test', json.dumps({'value': value}).encode())
        await recorder._flush()
    await recorder.stop()
    return sorted(recorder.output_dir.glob(f'{run}_[0-9]*.mcap'))


def _values(path: Path) -> list:
    with open(path, 'rb') as f:
        return [json.loads(message.data)['value'] for _, _, message in make_reader(f).iter_messages()]


def _records(path: Path) -> dict[str, dict]:
    """Read every metadata record of a recording.

    :param path: the MCAP file to read.
    :return: the decoded JSON payload of each record, by record name.
    """
    with open(path, 'rb') as f:
        return {record.name: json.loads(record.metadata['json']) for record in make_reader(f).iter_metadata()}


def _files(mcap_dir: Path) -> set[str]:
    return {path.name for path in mcap_dir.iterdir()}


async def test_a_merged_recording_keeps_the_context_of_its_sources(mcap_dir: Path) -> None:
    """Merging must not strip the metadata; the merged file is the one people send around."""
    parts = await _record(_recorder(mcap_dir), [[0], [1]], metadata={'mission': 'Implement Demo', 'run_id': 42})

    target = mcap_dir / 'merged.mcap'
    count = merge_recordings(parts, target)

    assert count == 2
    assert _values(target) == [0, 1]
    assert _records(target)[METADATA_NAME] == {'mission': 'Implement Demo', 'run_id': 42}


async def test_a_merged_recording_says_how_it_came_to_be(mcap_dir: Path) -> None:
    """The merge leaves its own record, so nobody wonders where a file came from."""
    parts = await _record(_recorder(mcap_dir), [[0]])

    target = mcap_dir / 'merged.mcap'
    merge_recordings(parts, target)

    assert _records(target)[MERGE_METADATA_NAME]['sources'] == [parts[0].name]
    assert _records(target)[MERGE_METADATA_NAME]['trimmed'] is False


async def test_a_merge_can_start_at_a_given_time(mcap_dir: Path) -> None:
    """Messages before the start time are left out, and the merge record says it was trimmed."""
    recorder = _recorder(mcap_dir)
    started_at = rosys.time()
    parts = await _record(recorder, [[0, 1], [2, 3]])

    target = mcap_dir / 'merged.mcap'
    count = merge_recordings(parts, target, start_time_ns=int((started_at + 30) * NS))

    assert count == 2
    assert _values(target) == [2, 3]
    assert _records(target)[MERGE_METADATA_NAME]['trimmed'] is True


async def test_runs_with_different_metadata_each_keep_their_record(mcap_dir: Path) -> None:
    """A merge across runs keeps the context of every run, numbered apart."""
    recorder = _recorder(mcap_dir)
    first = await _record(recorder, [[0]], metadata={'run_id': 1})
    second = await _record(recorder, [[1]], metadata={'run_id': 2})

    target = mcap_dir / 'merged.mcap'
    merge_recordings(first + second, target)

    records = _records(target)
    assert records[METADATA_NAME] == {'run_id': 1}
    assert records[f'{METADATA_NAME}_2'] == {'run_id': 2}


async def test_merging_merged_recordings_keeps_every_context(mcap_dir: Path) -> None:
    """A merged file merged again still names every run it holds and every merge it went through."""
    recorder = _recorder(mcap_dir)
    first = await recorder.merge(await _record(recorder, [[0], [1]], metadata={'run_id': 1}), 'first')
    second = await recorder.merge(await _record(recorder, [[2]], metadata={'run_id': 2}), 'second')
    assert first is not None and second is not None

    both = await recorder.merge([first, second], 'both')

    assert both is not None
    assert _values(both) == [0, 1, 2]
    records = _records(both)
    assert [records[name] for name in sorted(records) if name.startswith(METADATA_NAME)] == \
        [{'run_id': 1}, {'run_id': 2}]
    assert records[MERGE_METADATA_NAME]['sources'] == ['first.mcap', 'second.mcap']
    assert sorted(len(records[name]['sources']) for name in records if name.startswith(f'{MERGE_METADATA_NAME}_')) \
        == [1, 2]


async def test_a_topic_keeps_the_schema_each_source_recorded_it_with(mcap_dir: Path) -> None:
    """The same topic recorded before and after a schema change stays readable in both forms."""
    recorder = _recorder(mcap_dir)
    before = await _record(recorder, [[0]])
    recorder.add_topic('/test', _schema({'value': {'type': 'integer'}}))
    after = await _record(recorder, [[1]])

    target = mcap_dir / 'merged.mcap'
    merge_recordings(before + after, target)

    with open(target, 'rb') as f:
        schemas = [json.loads(schema.data)['properties']['value']['type']
                   for schema, _, _ in make_reader(f).iter_messages()]
    assert schemas == ['number', 'integer']


async def test_a_run_merges_into_a_kept_recording_that_replaces_its_parts(mcap_dir: Path) -> None:
    """The merged run is one kept file; its parts are gone once it is in place."""
    recorder = _recorder(mcap_dir)
    parts = await _record(recorder, [[0], [1], [2]], name='mission')

    target = await recorder.merge(parts, 'mission_merged')

    assert target == mcap_dir / 'mission_merged.mcap'
    assert _files(mcap_dir) == {'mission_merged.mcap'}
    assert _values(target) == [0, 1, 2]
    assert not is_auto_named(target)


async def test_a_run_with_an_unindexed_part_can_be_merged(mcap_dir: Path) -> None:
    """A part left without its index by a crash is reindexed and merged with its context."""
    recorder = _recorder(mcap_dir)
    [part] = await _record(recorder, [[0]], metadata={'run_id': 1})
    unfinished = part.with_name(part.name.replace('_01.mcap', '_02.mcap'))
    with open(unfinished, 'wb') as f:
        writer = Writer(f, chunk_size=1)  # every message closes a chunk, so all of them are on disk
        writer.start()
        writer.add_metadata(METADATA_NAME, {'json': json.dumps({'run_id': 1, 'crashed': True})})
        channel = writer.register_channel('/test', 'json', writer.register_schema('Test', 'jsonschema', b'{}'))
        log_time = int((rosys.time() + 1) * NS)
        writer.add_message(channel, log_time=log_time, data=b'{"value": 1}', publish_time=log_time)
        # never finished, as after a crash: no summary index

    target = await recorder.merge([part, unfinished], 'merged')

    assert target is not None
    assert _values(target) == [0, 1]
    assert _records(target)[f'{METADATA_NAME}_2'] == {'run_id': 1, 'crashed': True}


async def test_a_second_merge_into_the_same_name_is_refused(mcap_dir: Path) -> None:
    """Two clients merging the same run at once cannot write into the same file."""
    recorder = _recorder(mcap_dir)
    parts = await _record(recorder, [[0], [1]])

    first, second = await asyncio.gather(recorder.merge(parts, 'merged'), recorder.merge(parts, 'merged'),
                                         return_exceptions=True)

    assert first == mcap_dir / 'merged.mcap'
    assert isinstance(second, FileExistsError)
    assert _values(mcap_dir / 'merged.mcap') == [0, 1]
    assert recorder.merging == frozenset()


async def test_a_merge_never_overwrites_a_recording(mcap_dir: Path) -> None:
    """An existing recording under the merged name stays as it is, and so do the sources."""
    recorder = _recorder(mcap_dir)
    parts = await _record(recorder, [[0], [1]])
    existing = mcap_dir / 'merged.mcap'
    existing.write_bytes(b'a recording of its own')

    with pytest.raises(FileExistsError):
        await recorder.merge(parts, 'merged')
    with pytest.raises(FileExistsError):
        merge_into_place(parts, existing)

    assert existing.read_bytes() == b'a recording of its own'
    assert _files(mcap_dir) == {existing.name, *(part.name for part in parts)}


async def test_a_failed_merge_keeps_its_sources(mcap_dir: Path) -> None:
    """Sources go only once the merged file is in place; a merge that fails leaves them be."""
    recorder = _recorder(mcap_dir)
    parts = await _record(recorder, [[0], [1]])
    vanished = parts[-1].with_name(parts[-1].name.replace('_02.mcap', '_03.mcap'))

    with pytest.raises(FileNotFoundError):
        await recorder.merge([*parts, vanished], 'merged')

    assert _files(mcap_dir) == {part.name for part in parts}


async def test_a_merge_with_nothing_left_to_merge_keeps_its_sources(mcap_dir: Path) -> None:
    """A start time after the last message merges nothing, so no file is written and the sources stay."""
    recorder = _recorder(mcap_dir)
    parts = await _record(recorder, [[0], [1]])

    target = await recorder.merge(parts, 'merged', start_time_ns=int((rosys.time() + 3600) * NS))

    assert target is None
    assert _files(mcap_dir) == {part.name for part in parts}


@pytest.mark.parametrize('name', ['20260911_120000_123456_merged_01', '../merged', 'sub/merged'])
async def test_a_merged_name_must_be_a_kept_file_name(mcap_dir: Path, name: str) -> None:
    """A merged recording cannot land outside the directory or read as a file the budget deletes first."""
    recorder = _recorder(mcap_dir)
    parts = await _record(recorder, [[0], [1]])

    with pytest.raises(ValueError):
        await recorder.merge(parts, name)

    assert _files(mcap_dir) == {part.name for part in parts}


async def test_the_live_recording_cannot_be_merged(mcap_dir: Path) -> None:
    """The file being written is no finished recording yet."""
    recorder = _recorder(mcap_dir)
    recorder.start()
    live = recorder.current_recording
    assert live is not None

    with pytest.raises(ValueError):
        await recorder.merge([live], 'merged')

    await recorder.stop()
