import tempfile
from pathlib import Path

import pytest
from fastapi import status
from fastapi.responses import FileResponse, JSONResponse

from rosys.analysis.recording import McapRecorder, RecordingInfo, TopicSchema
from rosys.analysis.recording.recordings_page_ import _download_response, _group_by_run, _replace_sources

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


def _info(name: str, *, mtime: float = 0.0, size: int = 0) -> RecordingInfo:
    """A snapshot of a recording, as the page's scan hands it to the grouping.

    :param name: the file name the grouping reads the run key off.
    :param mtime: the modification time shown in the list.
    :param size: the file size shown in the list.
    :return: the snapshot to group.
    """
    return RecordingInfo(Path(name), mtime, size, False, True)


def test_the_parts_of_a_run_form_one_entry() -> None:
    """The numbered files of one run are grouped under the name they share."""
    parts = [_info('20260911_052815_run0002_02.mcap'), _info('20260911_052815_run0002_01.mcap')]

    assert _group_by_run(parts) == [('20260911_052815_run0002', parts)]


def test_runs_stay_apart_and_keep_their_order() -> None:
    """Each run gets its own entry, in the order its newest file appears in the list."""
    newer = _info('20260911_052815_run0002_01.mcap')
    older = [_info('20260910_120000_run0001_02.mcap'), _info('20260910_120000_run0001_01.mcap')]

    assert _group_by_run([newer, *older]) == [
        ('20260911_052815_run0002', [newer]),
        ('20260910_120000_run0001', older),
    ]


@pytest.mark.parametrize('name', ['failure_20260911_052815_123456.mcap',  # preserved around a failure
                                  '20260911_052815_123456_01.mcap',  # recorded without a run name
                                  '20260911_052815_run0002.mcap',  # the merged file of a run
                                  'weeding on the north field.mcap'])  # renamed by hand
def test_a_file_without_a_part_number_stays_on_its_own(name: str) -> None:
    """Anything the recorder did not number apart as a run's part keeps its own entry."""
    info = _info(name)

    assert _group_by_run([info]) == [(None, [info])]


def test_a_merge_only_takes_the_run_name_once_it_is_complete(tmp_path: Path) -> None:
    """A merge in progress must not look like a recording, and the parts go only after it lands."""
    unfinished = tmp_path / 'run.mcap.part'
    unfinished.write_bytes(b'merged')
    sources = [tmp_path / 'run_01.mcap', tmp_path / 'run_02.mcap']
    for source in sources:
        source.write_bytes(b'part')

    assert sorted(path.name for path in tmp_path.glob('*.mcap')) == ['run_01.mcap', 'run_02.mcap']

    _replace_sources(unfinished, tmp_path / 'run.mcap', sources)

    assert (tmp_path / 'run.mcap').read_bytes() == b'merged'
    assert not unfinished.exists()
    assert not any(source.exists() for source in sources)
