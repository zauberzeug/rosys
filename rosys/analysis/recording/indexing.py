"""Check and rebuild the MCAP summary index of recordings.

A recording is only indexed once the writer's ``finish()`` has run; a file left
behind by a crashed process has no summary section and Foxglove reports it as
"unindexed". :func:`reindex` rewrites such a file with an index, recovering every
readable message (all but the final unflushed chunk).
"""
from __future__ import annotations

from pathlib import Path
from struct import error as StructError
from uuid import uuid4

from mcap.exceptions import McapError
from mcap.reader import make_reader
from mcap.records import Channel, Header, Message, Schema
from mcap.stream_reader import StreamReader
from mcap.writer import CompressionType, Writer
from zstandard import ZstdError

# Errors raised while reading past the last cleanly written record of a crashed
# recording: the truncated/half-flushed tail decodes into a bogus length, a broken
# zstd frame, or non-UTF-8 bytes. Everything readable up to that point has already
# been recovered, so hitting one of these means "stop", not "fail". Write-side
# errors are deliberately excluded and propagate (see :func:`reindex`).
_TRUNCATION_ERRORS = (McapError, StructError, ZstdError, UnicodeDecodeError, OverflowError)


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

    Only read-side failures on the crashed tail are tolerated; any write-side
    failure (e.g. ENOSPC while the temp copy transiently doubles disk usage)
    removes the temp file and re-raises, so the original complete file is never
    replaced by a truncated one.

    The rebuild goes through a per-run unique temp file (see :func:`_reindex_temp_path`),
    so two overlapping reindexes of the same recording cannot interleave writes and
    corrupt each other's output.

    :param path: the recording to reindex in place.
    :param chunk_size: the target chunk size of the rewritten file.
    :return: the number of messages recovered into the indexed copy.
    :raises Exception: if writing the indexed copy fails; the original is left untouched.
    """
    path = Path(path)
    temp = _reindex_temp_path(path)
    count = 0
    try:
        with open(path, 'rb') as source, open(temp, 'wb') as target:
            writer = Writer(target, compression=CompressionType.ZSTD, chunk_size=chunk_size)
            started = False
            schemas: dict[int, int] = {}
            channels: dict[int, int] = {}
            records = iter(StreamReader(source, emit_chunks=False).records)
            while True:
                try:
                    record = next(records)
                except StopIteration:
                    break
                except _TRUNCATION_ERRORS:
                    break  # reached the unflushed tail of a crashed recording; keep the readable prefix
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
            if not started:
                writer.start(profile='', library='rosys-reindex')
            writer.finish()
    except BaseException:  # a write-side failure must not leave a truncated temp (nor replace the original)
        temp.unlink(missing_ok=True)
        raise
    temp.replace(path)
    return count


def _reindex_temp_path(path: Path) -> Path:
    """A unique temp path for one reindex run, alongside ``path`` so ``replace()`` stays atomic.

    Each run gets its own random suffix, so two overlapping reindexes of the same
    recording write to different temp files and cannot corrupt each other's output.
    Orphaned temps left by a crash mid-rebuild are cleaned up on recorder startup
    (glob ``*.mcap.reindex*``), so the suffix must keep the ``.mcap.reindex`` prefix.

    :param path: the recording being reindexed.
    :return: a sibling path named ``<recording>.reindex-<random hex>``.
    """
    return path.with_name(f'{path.name}.reindex-{uuid4().hex}')
