import tempfile
from pathlib import Path

import pytest
from fastapi import status
from fastapi.responses import FileResponse, JSONResponse

from rosys.analysis.recording import McapRecorder, TopicSchema
from rosys.analysis.recording.recordings_page_ import _download_response

# The download endpoint is registered as a closure on the global nicegui ``app`` once per
# RecordingsPage instance (path-matched first-registration-wins), and exercising it over HTTP
# would run nicegui's full app lifespan. So these tests drive the extracted ``_download_response``
# guard directly against a real recorder and its on-disk output directory — the same code the
# endpoint runs, without cross-test route accumulation or a running server.


@pytest.fixture
def recorder():
    with tempfile.TemporaryDirectory(prefix='rosys-mcap-') as tmp:
        yield McapRecorder(output_dir=Path(tmp), auto_start=False)


def _make_recording(recorder: McapRecorder, name: str) -> Path:
    path = recorder.output_dir / name
    path.write_bytes(b'not really mcap, but a real file on disk')
    return path


def test_download_serves_a_finished_recording(recorder: McapRecorder) -> None:
    """A valid .mcap name returns the file with a 200 response."""
    path = _make_recording(recorder, 'recording.mcap')

    response = _download_response(recorder, 'recording.mcap')

    assert isinstance(response, FileResponse)
    assert Path(response.path) == path
    assert response.status_code == status.HTTP_200_OK


def test_download_404_for_missing_recording(recorder: McapRecorder) -> None:
    """A well-formed name with no file on disk returns 404."""
    response = _download_response(recorder, 'does-not-exist.mcap')

    assert isinstance(response, JSONResponse)
    assert response.status_code == status.HTTP_404_NOT_FOUND


@pytest.mark.parametrize('name', ['../../etc/passwd', '/etc/passwd'])
def test_download_404_for_path_traversal(recorder: McapRecorder, name: str) -> None:
    """A traversal or absolute path is reduced to its basename and cannot escape the output directory."""
    response = _download_response(recorder, name)

    assert isinstance(response, JSONResponse)
    assert response.status_code == status.HTTP_404_NOT_FOUND


def test_download_404_for_traversal_that_ends_in_mcap(recorder: McapRecorder) -> None:
    """A .mcap traversal target resolves to the basename inside the output directory, which is missing."""
    response = _download_response(recorder, '../secret.mcap')

    assert isinstance(response, JSONResponse)
    assert response.status_code == status.HTTP_404_NOT_FOUND


def test_download_404_for_non_mcap_name(recorder: McapRecorder) -> None:
    """A name without the .mcap suffix is never served."""
    _make_recording(recorder, 'notes.txt')

    response = _download_response(recorder, 'notes.txt')

    assert isinstance(response, JSONResponse)
    assert response.status_code == status.HTTP_404_NOT_FOUND


def test_download_404_for_reindex_temp_name(recorder: McapRecorder) -> None:
    """A transient .reindex- temp file is never served even though it exists on disk."""
    _make_recording(recorder, 'recording.mcap.reindex-deadbeef')

    response = _download_response(recorder, 'recording.mcap.reindex-deadbeef')

    assert isinstance(response, JSONResponse)
    assert response.status_code == status.HTTP_404_NOT_FOUND


async def test_download_409_for_the_live_recording(recorder: McapRecorder) -> None:
    """The file currently being written cannot be downloaded while the writer holds it open."""
    recorder.add_topic('/t', TopicSchema('T', b'{}', 'jsonschema', 'json'))
    recorder.start()
    try:
        live = recorder.current_recording
        assert live is not None

        response = _download_response(recorder, live.name)

        assert isinstance(response, JSONResponse)
        assert response.status_code == status.HTTP_409_CONFLICT
    finally:
        await recorder.stop()


def test_download_404_for_name_with_nul_byte(recorder: McapRecorder) -> None:
    """A NUL byte in the name is rejected up front, not passed to is_file() (which would 500)."""
    response = _download_response(recorder, 'recording\x00.mcap')

    assert isinstance(response, JSONResponse)
    assert response.status_code == status.HTTP_404_NOT_FOUND
