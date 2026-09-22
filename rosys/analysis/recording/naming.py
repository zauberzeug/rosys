"""How recordings are named, and how a name is read back."""
from __future__ import annotations

import re
from pathlib import Path

TIMESTAMP_FORMAT = r'%Y%m%d_%H%M%S_%f'  # microseconds -> unique per run

# part <part> of the run <timestamp> or <timestamp>_<name>, or a single unnumbered <timestamp>.mcap
_OWN_NAME = re.compile(r'(?P<run>\d{8}_\d{6}_\d{6}(?:_.+)?)_(?P<part>\d{2,})\.mcap|\d{8}_\d{6}_\d{6}\.mcap')


def is_auto_named(path: Path | str) -> bool:
    """Whether a recording is one the recorder wrote itself: ``<run>_<part>.mcap``, the run named by its start.

    Any other name was given deliberately (a renamed, merged or preserved recording); the budget keeps it longer.

    :param path: the recording file to test.
    :return: ``True`` if the file name is one the recorder generated.
    """
    return _OWN_NAME.fullmatch(Path(path).name) is not None


def run_and_part(path: Path | str) -> tuple[str, int] | None:
    """Read which run a recording is a part of, and which part it is.

    :param path: the recording file to read.
    :return: the run's name and the part's number, or ``None`` for a file that is no numbered part.
    """
    match = _OWN_NAME.fullmatch(Path(path).name)
    if match is None or match.group('run') is None:
        return None
    return match.group('run'), int(match.group('part'))


def check_file_name(name: str) -> None:
    """Refuse a name that is not a plain file name, so no recording lands outside the output directory.

    :param name: the name to check.
    :raises ValueError: if the name is empty, whitespace or dots only, or contains a path separator.
    """
    if Path(name).name != name or not name.strip('.').strip():
        raise ValueError(f'not a plain file name: {name!r}')
