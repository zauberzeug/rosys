"""How recordings are named, and how a name is read back."""
from __future__ import annotations

import re
from datetime import UTC, datetime
from pathlib import Path

TIMESTAMP_FORMAT = r'%Y%m%d_%H%M%S_%f'  # microseconds -> unique per run
TIMESTAMP_LENGTH = 22

# part <part> of the run <timestamp> or <timestamp>_<name>
_PART_NAME = re.compile(r'(?P<run>\d{8}_\d{6}_\d{6}(?:_.+)?)_(?P<part>\d{2,})\.mcap')


def run_and_part(path: Path | str) -> tuple[str, int] | None:
    """Read which run a recording is a part of, and which part it is.

    :param path: the recording file to read.
    :return: the run's name and the part's number, or ``None`` for a file that is no numbered part.
    """
    match = _PART_NAME.fullmatch(Path(path).name)
    if match is None:
        return None
    return match.group('run'), int(match.group('part'))


def run_start(run: str) -> datetime:
    """When a run started, read from the timestamp its name begins with.

    :param run: the run's name, ``<timestamp>`` or ``<timestamp>_<name>``.
    :return: the start as an aware UTC datetime.
    """
    return datetime.strptime(run[:TIMESTAMP_LENGTH], TIMESTAMP_FORMAT).replace(tzinfo=UTC)


def ensure_plain_file_name(name: str) -> None:
    """Refuse a name that is not a plain file name, so no recording lands outside the output directory.

    :param name: the name to check.
    :raises ValueError: if the name is empty, whitespace or dots only, or contains a path separator.
    """
    if Path(name).name != name or not name.strip('.').strip():
        raise ValueError(f'not a plain file name: {name!r}')
