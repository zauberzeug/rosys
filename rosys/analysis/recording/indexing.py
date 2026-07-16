"""Check and rebuild the MCAP summary index of recordings.

A recording is only indexed once the writer's ``finish()`` has run; a file left
behind by a crashed process has no summary section and Foxglove reports it as
"unindexed". :func:`reindex` rewrites such a file with an index, recovering every
readable message (all but the final unflushed chunk).
"""
from __future__ import annotations

from pathlib import Path

from mcap.reader import make_reader
from mcap.records import Channel, Header, Message, Schema
from mcap.stream_reader import StreamReader
from mcap.writer import CompressionType, Writer


def is_indexed(path: Path | str) -> bool:
    """Whether the file has an MCAP summary section (a seekable index)."""
    try:
        with open(path, 'rb') as file:
            return make_reader(file).get_summary() is not None
    except Exception:  # pylint: disable=broad-except  # truncated/unfinished files raise instead of None
        return False


def reindex(path: Path | str, *, chunk_size: int = 1_048_576) -> int:
    """Rewrite ``path`` in place with a summary index; return the recovered message count.

    Reads the file sequentially (tolerating a truncated tail from a crash) and
    writes a fresh, indexed copy. Messages in the final unflushed chunk of a
    crashed recording are unrecoverable.
    """
    path = Path(path)
    temp = path.with_name(path.name + '.reindex')
    count = 0
    with open(path, 'rb') as source, open(temp, 'wb') as target:
        writer = Writer(target, compression=CompressionType.ZSTD, chunk_size=chunk_size)
        started = False
        schemas: dict[int, int] = {}
        channels: dict[int, int] = {}
        try:
            for record in StreamReader(source, emit_chunks=False).records:
                if isinstance(record, Header):
                    writer.start(profile=record.profile, library=record.library)
                    started = True
                elif isinstance(record, Schema):
                    schemas[record.id] = writer.register_schema(
                        name=record.name, encoding=record.encoding, data=record.data)
                elif isinstance(record, Channel):
                    channels[record.id] = writer.register_channel(
                        schema_id=schemas.get(record.schema_id, 0), topic=record.topic,
                        message_encoding=record.message_encoding, metadata=record.metadata)
                elif isinstance(record, Message) and record.channel_id in channels:
                    writer.add_message(
                        channel_id=channels[record.channel_id], log_time=record.log_time,
                        data=record.data, publish_time=record.publish_time, sequence=record.sequence)
                    count += 1
        except Exception:  # pylint: disable=broad-except  # truncated tail of a crashed recording
            pass
        if not started:
            writer.start(profile='', library='rosys-reindex')
        writer.finish()
    temp.replace(path)
    return count
