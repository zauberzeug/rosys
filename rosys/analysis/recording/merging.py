"""Merge MCAP recordings into one file, keeping the context they were recorded in."""
import json
import os
from collections import defaultdict
from collections.abc import Sequence
from contextlib import ExitStack
from datetime import UTC, datetime
from pathlib import Path
from typing import BinaryIO
from uuid import uuid4

from mcap.reader import make_reader
from mcap.records import Chunk, DataEnd, Metadata
from mcap.stream_reader import StreamReader
from mcap.writer import CompressionType, Writer

from .indexing import is_indexed, reindex

METADATA_NAME = 'recording'
"""Metadata record holding the caller's context of a recording as JSON."""

MERGE_METADATA_NAME = 'merge'
"""Metadata record describing the merge itself, so a merged file says how it came to be."""

STAGING_GLOB = '*.mcap.merge-*'
"""The files a merge writes before the result takes its name; not ``*.mcap``, so no listing shows them."""


def merge_into_place(sources: list[Path], target: Path, *, start_time_ns: int = 0) -> tuple[int, list[Path]]:
    """Merge ``sources`` into ``target`` and delete them once the merged file is in place.

    Unindexed sources (e.g. left by a crash) are reindexed first. The merge writes to a staging
    file beside ``target`` and only a finished file takes the target name, refusing an existing
    one, so an interrupted or failed merge leaves the sources untouched.

    Blocking I/O and ZSTD recompression; call via ``rosys.run.io_bound``.

    :param sources: the recordings to merge, oldest first.
    :param target: the file the merged recording takes.
    :param start_time_ns: messages logged before this time are dropped (default: keep all).
    :return: the number of messages merged, ``0`` leaving no target and the sources in place, and
        the sources that could not be deleted.
    :raises FileExistsError: if ``target`` exists; the sources are kept.
    """
    for source in sources:
        if not is_indexed(source):
            reindex(source)
    staging = target.with_name(f'{target.name}.merge-{uuid4().hex}')
    try:
        count = merge_recordings(sources, staging, start_time_ns=start_time_ns)
        if count:
            os.link(staging, target)  # atomic: fails if the target exists, so no recording is overwritten
    finally:
        staging.unlink(missing_ok=True)
    if not count:
        return 0, []
    undeleted: list[Path] = []
    for source in sources:
        try:
            source.unlink(missing_ok=True)
        except OSError:
            undeleted.append(source)
    return count, undeleted


def merge_recordings(sources: list[Path], target: Path, *, start_time_ns: int = 0) -> int:
    """Write the messages of ``sources`` (in the given order) into ``target``.

    Every metadata record of the sources is carried over, so a merged recording still says which
    robot, run and mission it belongs to. Identical records are written once; records of the same
    name with different payloads are numbered ``<name>``, ``<name>_2``, ... (the merges a merged
    source went through start at ``merge_2``, leaving ``merge`` to the merge at hand).

    All sources are opened before anything is written, so a source the recorder's disk budget
    deletes meanwhile stays readable and still lands in the result.

    Every source needs its summary index (see :func:`~.indexing.reindex`). Blocking I/O and
    ZSTD recompression; call via ``rosys.run.io_bound``.

    :param sources: the recordings to merge, oldest first.
    :param target: the file to write.
    :param start_time_ns: messages logged before this time are dropped (default: keep all).
    :return: the number of messages written.
    """
    count = 0
    schema_ids: dict[tuple[str, str, bytes], int] = {}
    channel_ids: dict[tuple[str, str, int], int] = {}
    with ExitStack() as stack:
        streams = [stack.enter_context(open(source, 'rb')) for source in sources]
        with open(target, 'wb') as file:
            writer = Writer(file, compression=CompressionType.ZSTD)
            writer.start(profile='rosys', library='rosys-merge')
            for name, payload in _collected_metadata(streams).items():
                writer.add_metadata(name, payload)
            writer.add_metadata(MERGE_METADATA_NAME, {'json': json.dumps({
                'merged_at': datetime.now(tz=UTC).isoformat(),
                'sources': [source.name for source in sources],
                'trimmed': bool(start_time_ns),
            })})
            for stream in streams:
                stream.seek(0)
                for schema, channel, message in make_reader(stream).iter_messages(start_time=start_time_ns):
                    schema_id = 0
                    if schema is not None:
                        schema_key = (schema.name, schema.encoding, bytes(schema.data))
                        if schema_key not in schema_ids:
                            schema_ids[schema_key] = writer.register_schema(schema.name, schema.encoding, schema.data)
                        schema_id = schema_ids[schema_key]
                    channel_key = (channel.topic, channel.message_encoding, schema_id)
                    if channel_key not in channel_ids:
                        channel_ids[channel_key] = writer.register_channel(
                            channel.topic, channel.message_encoding, schema_id)
                    writer.add_message(channel_ids[channel_key], message.log_time, message.data,
                                       message.publish_time)
                    count += 1
            writer.finish()
    return count


def _collected_metadata(streams: Sequence[BinaryIO]) -> dict[str, dict[str, str]]:
    """The distinct metadata records of ``streams``, named for the merged file.

    Records of the same name with identical payloads come out once; differing payloads are
    numbered ``<name>``, ``<name>_2``, ... in the order they were found (a source's ``recording_2``
    counts as a ``recording``). The ``merge`` records of the sources start at ``merge_2``,
    leaving ``merge`` to the merge at hand.

    :param streams: the open recordings to read; each is rewound first.
    :return: the records to write into the merged file, by name.
    """
    found: dict[str, list[dict[str, str]]] = defaultdict(list)
    for stream in streams:
        stream.seek(0)
        for record in StreamReader(stream, emit_chunks=True).records:
            if isinstance(record, (Chunk, DataEnd)):
                break  # metadata sits ahead of the first chunk, so nothing is decompressed
            if isinstance(record, Metadata) and record.metadata not in found[_base_name(record.name)]:
                found[_base_name(record.name)].append(record.metadata)
    collected: dict[str, dict[str, str]] = {}
    for base, payloads in found.items():
        first = 2 if base == MERGE_METADATA_NAME else 1
        for number, payload in enumerate(payloads, start=first):
            collected[base if number == 1 else f'{base}_{number}'] = payload
    return collected


def _base_name(name: str) -> str:
    """The name a numbered record was numbered from, e.g. ``recording`` for ``recording_2``.

    :param name: the name of a metadata record.
    :return: the name without its number.
    """
    base, _, number = name.rpartition('_')
    return base if base and number.isdigit() else name
