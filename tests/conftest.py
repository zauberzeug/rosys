import tempfile
from collections.abc import Iterator
from pathlib import Path

import pytest

from rosys import rosys as rosys_core
from rosys.testing.fixtures import *  # pylint: disable=wildcard-import,unused-wildcard-import # noqa: F403


@pytest.fixture
def recordings_dir():
    with tempfile.TemporaryDirectory(prefix='rosys-test-', ignore_cleanup_errors=True) as tmp_path:
        d = Path(tmp_path) / 'recordings'
        d.mkdir()
        yield d


@pytest.fixture
def mcap_dir() -> Iterator[Path]:
    with tempfile.TemporaryDirectory(prefix='rosys-mcap-') as tmp:
        yield Path(tmp)


@pytest.fixture(autouse=True)
def _restore_rosys_handlers() -> Iterator[None]:
    """Restore rosys's global startup/shutdown handler lists around every test.

    ``McapRecorder.__init__`` registers a repeat handler (via ``rosys.on_repeat``) and a
    shutdown handler (``rosys.on_shutdown``) on rosys's module-global handler lists. Tests
    that construct a recorder without the ``rosys_integration`` fixture never call
    ``reset_after_test``, which is the only thing that clears those lists, so the handlers
    would otherwise accumulate across the whole session and all fire together in the first
    integration test that runs ``rosys.startup``/``rosys.shutdown`` afterwards. Snapshotting
    and restoring the list contents keeps each test's registrations from leaking into the
    next, while leaving the ``rosys_integration`` tests (which reset themselves) untouched.
    """
    startup_handlers = rosys_core.startup_handlers[:]
    shutdown_handlers = rosys_core.shutdown_handlers[:]
    tasks = rosys_core.tasks[:]
    yield
    rosys_core.startup_handlers[:] = startup_handlers
    rosys_core.shutdown_handlers[:] = shutdown_handlers
    rosys_core.tasks[:] = tasks
