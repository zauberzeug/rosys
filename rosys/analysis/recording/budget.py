"""Which recordings the disk budget of :class:`~.mcap_recorder.McapRecorder` deletes.

Described in ``docs/examples/mcap_recording/README.md`` under "Disk budget" and "Protecting kept recordings".
"""
from __future__ import annotations

from pathlib import Path
from typing import NamedTuple


class StoredRecording(NamedTuple):
    path: Path
    size: int
    mtime: float
    kept: bool
    protected: bool


class Deletions(NamedTuple):
    unprotected: list[StoredRecording]
    protected: list[StoredRecording]
    """deleted only because the unprotected ones were not enough to meet the total"""


def select_deletions(recordings: list[StoredRecording], *, max_kept_size: int, max_total_size: int) -> Deletions:
    """Pick the oldest recordings to delete until the kept ones fit ``max_kept_size`` and all fit ``max_total_size``.

    The kept bound spares the newest kept recording and every protected one; the total is then met oldest first
    across parts and kept recordings, by unprotected ones first and by protected ones only if that is not enough.

    :param recordings: every recording on disk.
    :param max_kept_size: bytes the kept recordings may take.
    :param max_total_size: bytes all recordings may take.
    :return: the recordings to delete.
    """
    by_age = sorted(recordings, key=lambda recording: recording.mtime)
    kept = [recording for recording in by_age if recording.kept]
    unprotected: list[StoredRecording] = []
    kept_size = sum(recording.size for recording in kept)
    for recording in kept[:-1]:  # the newest kept file is the one just filed away
        if kept_size <= max_kept_size:
            break
        if not recording.protected:
            unprotected.append(recording)
            kept_size -= recording.size
    remaining = [recording for recording in by_age if recording not in unprotected]
    total = sum(recording.size for recording in remaining)
    for recording in remaining:
        if total <= max_total_size:
            break
        if not recording.protected:
            unprotected.append(recording)
            total -= recording.size
    protected: list[StoredRecording] = []
    for recording in remaining:
        if total <= max_total_size:
            break
        if recording.protected:
            protected.append(recording)
            total -= recording.size
    return Deletions(unprotected, protected)
