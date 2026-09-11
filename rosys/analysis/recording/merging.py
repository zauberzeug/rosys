"""Merge MCAP recordings into one file, keeping the context they were recorded in."""
import json
import os
from datetime import UTC, datetime
from pathlib import Path
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

    The metadata of the sources is carried over, so a merged recording still says which
    robot, run and mission it belongs to. Identical records are written once; sources
    from different runs each keep their own, and so do the merges a merged source went through.

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
    with open(target, 'wb') as file:
        writer = Writer(file, compression=CompressionType.ZSTD)
        writer.start(profile='rosys', library='rosys-merge')
        for name, payload in _collected_metadata(sources).items():
            writer.add_metadata(name, payload)
        writer.add_metadata(MERGE_METADATA_NAME, {'json': json.dumps({
            'merged_at': datetime.now(tz=UTC).isoformat(),
            'sources': [source.name for source in sources],
            'trimmed': bool(start_time_ns),
        })})
        for source in sources:
            with open(source, 'rb') as stream:
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


def _collected_metadata(sources: list[Path]) -> dict[str, dict[str, str]]:
    """The distinct context and merge records of ``sources``, named for the merged file.

    A file carries its records ahead of its first chunk, so the scan stops there without
    decompressing any data. Context records are numbered ``recording``, ``recording_2``, ...;
    the merges a source went through keep their records as ``merge_2``, ``merge_3``, ...,
    leaving ``merge`` to the merge at hand.

    :param sources: the recordings to read.
    :return: the records to write into the merged file.
    """
    found: dict[str, list[dict[str, str]]] = {METADATA_NAME: [], MERGE_METADATA_NAME: []}
    for source in sources:
        with open(source, 'rb') as stream:
            for record in StreamReader(stream, emit_chunks=True).records:
                if isinstance(record, (Chunk, DataEnd)):
                    break
                if not isinstance(record, Metadata):
                    continue
                records = found.get(_base_name(record.name))
                if records is not None and record.metadata not in records:
                    records.append(record.metadata)
    collected = {METADATA_NAME if index == 0 else f'{METADATA_NAME}_{index + 1}': payload
                 for index, payload in enumerate(found[METADATA_NAME])}
    collected.update({f'{MERGE_METADATA_NAME}_{index + 2}': payload
                      for index, payload in enumerate(found[MERGE_METADATA_NAME])})
    return collected


def _base_name(name: str) -> str:
    """The name a numbered record was numbered from, e.g. ``recording`` for ``recording_2``.

    :param name: the name of a metadata record.
    :return: the name without its number.
    """
    base, _, number = name.rpartition('_')
    return base if base and number.isdigit() else name
