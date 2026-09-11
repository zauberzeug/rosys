"""Merge MCAP recordings into one file, keeping the context they were recorded in."""
import json
from datetime import UTC, datetime
from pathlib import Path

from mcap.reader import make_reader
from mcap.records import Metadata
from mcap.stream_reader import StreamReader
from mcap.writer import CompressionType, Writer

from .mcap_recorder import METADATA_NAME

MERGE_METADATA_NAME = 'merge'
"""Metadata record describing the merge itself, so a merged file says how it came to be."""


def merge_recordings(sources: list[Path], target: Path, *, start_time_ns: int = 0) -> int:
    """Write the messages of ``sources`` (in the given order) into ``target``.

    The metadata of the sources is carried over, so a merged recording still says which
    robot, run and mission it belongs to. Identical records are written once; sources
    from different runs each keep their own.

    Blocking I/O and ZSTD recompression; call via ``rosys.run.io_bound``.

    :param sources: the recordings to merge, oldest first.
    :param target: the file to write.
    :param start_time_ns: messages logged before this time are dropped (default: keep all).
    :return: the number of messages written.
    """
    count = 0
    schema_ids: dict[str, int] = {}
    channel_ids: dict[str, int] = {}
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
                    if schema is not None and schema.name not in schema_ids:
                        schema_ids[schema.name] = writer.register_schema(schema.name, schema.encoding, schema.data)
                    if channel.topic not in channel_ids:
                        schema_id = schema_ids[schema.name] if schema is not None else 0
                        channel_ids[channel.topic] = writer.register_channel(
                            channel.topic, channel.message_encoding, schema_id)
                    writer.add_message(channel_ids[channel.topic], message.log_time, message.data,
                                       message.publish_time)
                    count += 1
        writer.finish()
    return count


def _collected_metadata(sources: list[Path]) -> dict[str, dict[str, str]]:
    """The distinct metadata records of ``sources``, keyed by the name they are written under.

    Read with the streaming reader, which does not need the summary index a file gets on
    finishing, so an unfinished recording can be merged too. Sources sharing a record
    contribute it once; differing ones are numbered apart.

    :param sources: the recordings to read.
    :return: the records to write into the merged file.
    """
    collected: dict[str, dict[str, str]] = {}
    for source in sources:
        with open(source, 'rb') as stream:
            for record in StreamReader(stream).records:
                if not isinstance(record, Metadata) or record.name != METADATA_NAME:
                    continue
                if record.metadata in collected.values():
                    break  # already carried over by an earlier source
                collected[f'{record.name}_{len(collected) + 1}' if collected else record.name] = record.metadata
                break  # the recorder writes its record once, at the top of the file
    return collected
