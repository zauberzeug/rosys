import tempfile
from pathlib import Path

import pytest

from rosys.testing.fixtures import *  # pylint: disable=wildcard-import,unused-wildcard-import # noqa: F403


@pytest.fixture
def recordings_dir():
    with tempfile.TemporaryDirectory(prefix='rosys-test-', ignore_cleanup_errors=True, delete=False) as tmp_path:
        d = Path(tmp_path) / 'recordings'
        d.mkdir()
        yield d
