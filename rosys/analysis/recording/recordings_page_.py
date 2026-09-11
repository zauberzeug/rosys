from __future__ import annotations

import re
from collections.abc import Callable
from datetime import UTC, date, datetime, timedelta, timezone
from pathlib import Path

from fastapi import status
from fastapi.responses import FileResponse, JSONResponse, Response
from nicegui import app, ui

from ... import rosys
from .mcap_recorder import McapRecorder, RecordingInfo
from .merging import merge_recordings
from .paths import DOWNLOAD_PATH, PAGE_PATH

_MAX_TIMEZONE_OFFSET_MINUTES = 24 * 60  # reject offsets beyond ±24 h from untrusted client JavaScript

_RUN_PART = re.compile(r'(?P<run>.+_run\d+)_\d+\.mcap')  # one part of a run, e.g. 20260911_052815_run0002_01.mcap


class RecordingsPage:
    """Lists the MCAP recordings for download and deletion.

    A long recording rotates into many files, so the parts of one run are listed as a
    single entry that expands into its parts; a file without a run name stays a row of
    its own. The list refreshes whenever the recorder starts a new recording or stops
    one, can be filtered by date, and offers rebuilding the index of crash-orphaned
    (unindexed) recordings. All filesystem access (glob, stat, index check) runs
    off the event loop via ``rosys.run.io_bound``; the render reads only a cached
    snapshot, so opening the page never blocks the loop on disk I/O.

    A download endpoint at ``DOWNLOAD_PATH/{name}`` serves finished recordings over
    HTTP (basename only, refusing the live file with 409 and missing files with 404),
    so recordings can be fetched without scp.
    """

    def __init__(self, recorder: McapRecorder, *, header: Callable[[], None] | None = None) -> None:
        """Register the recordings page and its download endpoint on the nicegui app.

        :param recorder: the recorder whose recordings this page lists, manages, and serves.
        :param header: optional callback rendered once at the top of the page before the
            content (e.g. a shared application header or navigation); ``None`` renders no header.
        """
        self.recorder = recorder
        self.header = header

        @ui.page(PAGE_PATH)
        async def page() -> None:
            if self.header is not None:
                self.header()
            await self.content()

        @app.get(f'{DOWNLOAD_PATH}/{{name}}')
        def download_recording(name: str) -> Response:
            return _download_response(recorder, name)

    async def content(self) -> None:
        recorder = self.recorder
        local_tz = await _browser_timezone()  # show times in the viewer's local zone (files stay UTC)
        infos: list[RecordingInfo] = []  # cached snapshot, refreshed off the loop by reload()

        def _in_range(info: RecordingInfo) -> bool:
            value = date_input.value
            if not value:
                return True
            parts = [part.strip() for part in value.split(' - ')]
            try:
                start, end = date.fromisoformat(parts[0]), date.fromisoformat(parts[-1])
            except ValueError:
                return True
            recorded = datetime.fromtimestamp(info.mtime, tz=local_tz).date()
            return start <= recorded <= end

        expanded_runs: set[str] = set()  # survives the refresh, which redraws the whole list
        merging_runs: set[str] = set()

        @ui.refreshable
        def recordings_list() -> None:
            reindex_button.set_enabled(any(not info.indexed and not info.is_live for info in infos))
            visible = [info for info in infos if _in_range(info)]
            if not visible:
                ui.label('No recordings.').classes('text-grey p-4')
                return
            with ui.column().classes('w-full gap-1'):
                for key, parts in _group_by_run(visible):
                    if key is None or len(parts) == 1:
                        _recording_row(parts[0])
                    else:
                        _run_row(key, parts)

        def _run_row(key: str, parts: list[RecordingInfo]) -> None:
            """Render the files of one run as one entry that expands into its parts.

            The entry keeps the shape of a single recording's row, so a run and a lone
            recording sit on the same edges; only the parts inside are indented.

            :param key: the name the run's files share.
            :param parts: the run's files, newest first.
            """
            first = datetime.fromtimestamp(min(part.mtime for part in parts), tz=local_tz)
            last = datetime.fromtimestamp(max(part.mtime for part in parts), tz=local_tz)
            size = sum(part.size for part in parts)
            is_expanded = key in expanded_runs

            def toggle() -> None:
                expanded_runs.symmetric_difference_update({key})
                recordings_list.refresh()

            with ui.row().classes('w-full items-center justify-between border-b py-1'):
                with ui.column().classes('gap-0'):
                    with ui.row().classes('items-center gap-1'):
                        ui.label(key).classes('font-mono')
                        if any(part.is_live for part in parts):
                            ui.badge('recording').props('color=red')
                    ui.label(f'{first:%Y-%m-%d %H:%M:%S} - {last:%H:%M:%S} · '
                             f'{size / 1_048_576:.1f} MB · {len(parts)} parts').classes('text-xs text-grey')
                with ui.row().classes('gap-1'):
                    if key in merging_runs:
                        ui.spinner(size='sm').tooltip('merging this run into one recording')
                    elif not any(part.is_live for part in parts):
                        ui.button(icon='merge', on_click=lambda k=key, p=list(parts): _start_merge(k, p)) \
                            .props('flat dense').tooltip('merge this run into one recording')
                    ui.button(icon='expand_less' if is_expanded else 'expand_more', on_click=toggle) \
                        .props('flat dense').tooltip('show the files of this run')
            if is_expanded:
                with ui.column().classes('w-full gap-1 pl-8'):
                    for part in parts:
                        _recording_row(part, with_date=False)

        def _recording_row(info: RecordingInfo, *, with_date: bool = True) -> None:
            """Render one recording.

            :param info: the recording to render.
            :param with_date: whether to date the row; the parts of a run carry the date in
                their group entry and only need the time of day.
            """
            modified = datetime.fromtimestamp(info.mtime, tz=local_tz) \
                .strftime('%Y-%m-%d %H:%M:%S' if with_date else '%H:%M:%S')
            with ui.row().classes('w-full items-center justify-between border-b py-1'):
                with ui.column().classes('gap-0'):
                    with ui.row().classes('items-center gap-1'):
                        ui.label(info.path.name).classes('font-mono')
                        if info.is_live:
                            ui.badge('recording').props('color=red')
                        elif not info.indexed:
                            ui.badge('unindexed').props('color=orange')
                    ui.label(f'{modified} · {info.size / 1_048_576:.1f} MB').classes('text-xs text-grey')
                with ui.row().classes('gap-1'):
                    if not info.is_live:
                        ui.button(icon='edit', on_click=lambda p=info.path: _rename(p)) \
                            .props('flat dense').tooltip('rename')
                    download = ui.button(icon='download',
                                         on_click=lambda p=info.path: ui.download(p)).props('flat dense')
                    if not info.indexed:
                        download.disable()
                        download.tooltip('available once the recording is stopped or reindexed')
                    if not info.is_live:  # the live file cannot be deleted (writer holds it open)
                        ui.button(icon='delete', on_click=lambda p=info.path: _delete(p)).props('flat dense color=red')

        def _start_merge(key: str, parts: list[RecordingInfo]) -> None:
            """Merge a run in the background, so closing the page does not cancel it.

            :param key: the run whose files are merged.
            :param parts: the run's files.
            """
            merging_runs.add(key)
            recordings_list.refresh()
            rosys.background_tasks.create(_merge_run(key, parts), name=f'merge {key}')

        async def _merge_run(key: str, parts: list[RecordingInfo]) -> None:
            """Write the run's files into one recording and drop the parts once it is written.

            The merge writes beside the recordings under a name the list ignores, and only a
            finished file takes the target name, so a merge in progress never shows up as a
            recording and an interruption leaves the parts untouched.

            :param key: the run whose files are merged.
            :param parts: the run's files.
            """
            target = recorder.output_dir / f'{key}.mcap'
            unfinished = recorder.output_dir / f'{key}.mcap.part'  # not a *.mcap, so the list ignores it
            sources = sorted(part.path for part in parts)
            try:
                count = await rosys.run.io_bound(merge_recordings, sources, unfinished)
                if not count:
                    raise RuntimeError('the merge wrote no messages')
                await rosys.run.io_bound(_replace_sources, unfinished, target, sources)
                rosys.notify(f'Merged {len(sources)} files into {target.name}', type='positive')
            except Exception as e:
                unfinished.unlink(missing_ok=True)
                rosys.notify(f'Could not merge {key}: {e}', type='negative')
            finally:
                merging_runs.discard(key)
                await reload()

        async def reload() -> None:
            infos[:] = await rosys.run.io_bound(recorder.scan_recordings) or []  # None on shutdown
            recordings_list.refresh()

        async def _delete(path: Path) -> None:
            if not await _confirm_dialog(f'Delete recording {path.name}?'):
                return
            await rosys.run.io_bound(recorder.delete_recording, path)  # unlink off the loop
            await reload()

        async def _reindex() -> None:
            reindex_button.disable()  # block a second concurrent rebuild (double-click or same client)
            try:
                await recorder.reindex_unindexed()
            finally:
                await reload()  # refreshes the list, which recomputes the button's enabled state

        async def _delete_all() -> None:
            if not await _confirm_dialog('Delete all recordings?'):
                return
            await rosys.run.io_bound(recorder.delete_all_recordings)  # unlink + glob off the loop
            await reload()

        async def _rename(path: Path) -> None:
            with ui.dialog() as dialog, ui.card():
                ui.label(f'Rename {path.name}')
                name_input = ui.input('New name', value=path.stem).classes('w-full')
                with ui.row():
                    ui.button('Cancel', on_click=dialog.close).props('flat')
                    ui.button('Rename', on_click=lambda: dialog.submit(name_input.value))
            new_name = await dialog
            if new_name:
                renamed = await rosys.run.io_bound(recorder.rename_recording, path, new_name)  # off the loop
                if renamed is None:
                    rosys.notify(f'Could not rename {path.name} (name already in use or invalid)', type='negative')
                await reload()

        with ui.column().classes('w-full gap-2'):
            with ui.row().classes('w-full items-center gap-2'):
                date_input = ui.date_input('Date range', range_input=True, on_change=recordings_list.refresh)
                ui.space()
                reindex_button = ui.button(icon='build', on_click=_reindex) \
                    .props('flat dense').tooltip('rebuild the index of unindexed recordings')
                ui.button(icon='delete_sweep', on_click=_delete_all) \
                    .props('flat dense color=red').tooltip('delete all recordings')
                ui.button(icon='refresh', on_click=reload).props('flat dense')
            # the page already insets its content; the scroll area would add a second one, only for the list
            with ui.scroll_area().classes('w-full') \
                    .props('content-style="padding-left: 0; padding-right: 0"') \
                    .style('max-height: 75vh'):
                recordings_list()

        def _change_token() -> tuple[tuple[Path, ...], Path | None]:
            """A cheap change token — the recording paths plus the live file.

            Both accessors glob the output directory, so this runs off the event loop.
            """
            return tuple(recorder.recordings), recorder.current_recording

        async def sync_if_changed() -> None:
            # Picks up start/stop, size-based rotation, and external add/remove. The
            # change token globs the directory, so it is computed off the loop (matching
            # the class docstring); only an actual change triggers the fuller stat/index
            # scan in reload(). A client-scoped timer (auto-removed on disconnect) avoids
            # subscribing to the process-lifetime recorder, so handlers don't accumulate
            # across page visits, and reloading only on a change keeps it off idle ticks.
            token = await rosys.run.io_bound(_change_token)
            if token is None:
                return  # recorder shut down mid-scan
            live = next((info.path for info in infos if info.is_live), None)
            if token != (tuple(info.path for info in infos), live):
                await reload()

        ui.timer(0.1, reload, once=True)  # populate the snapshot off the loop after the page is built
        ui.timer(2.0, sync_if_changed)


def _group_by_run(infos: list[RecordingInfo]) -> list[tuple[str | None, list[RecordingInfo]]]:
    """Group the files that belong to the same run, keeping the given order.

    A run's files are the ones the recorder numbered apart while rotating; every
    other file (an old recording, a preserved failure, a renamed one) forms an
    entry of its own.

    :param infos: the recordings to group, in display order.
    :return: one entry per run key, each with the run's files in the given order;
        ``None`` as the key for a file that belongs to no run.
    """
    entries: list[tuple[str | None, list[RecordingInfo]]] = []
    runs: dict[str, list[RecordingInfo]] = {}
    for info in infos:
        key = _run_key(info.path)
        if key is None:
            entries.append((None, [info]))
        elif key in runs:
            runs[key].append(info)
        else:
            runs[key] = [info]
            entries.append((key, runs[key]))
    return entries


def _run_key(path: Path) -> str | None:
    """The name the files of one run share — everything up to the part number.

    :param path: the recording to read the run key off.
    :return: the shared name, or ``None`` if the file is not a numbered part of a run.
    """
    match = _RUN_PART.fullmatch(path.name)
    return match.group('run') if match is not None else None


def _download_response(recorder: McapRecorder, name: str) -> Response:
    """Build the HTTP response for a recording download request.

    Serves a finished recording by basename only (no path traversal), refusing the
    live file with 409 and anything missing, non-``.mcap``, or containing a NUL byte
    with 404.

    :param recorder: the recorder whose output directory holds the recordings.
    :param name: the requested recording name from the URL (reduced to its basename).
    :return: a :class:`FileResponse` for a valid recording, else a JSON error response.
    """
    if '\x00' in name:  # a NUL byte makes is_file() raise ValueError -> reject up front (else an unhandled 500)
        return _not_found()
    file_path = recorder.output_dir / Path(name).name  # basename only, no path traversal
    if file_path.suffix != '.mcap':  # never serve transient *.mcap.reindex temp files
        return _not_found()
    if file_path == recorder.current_recording:
        return JSONResponse(content={'status': 'error', 'message': 'Recording is currently being written'},
                            status_code=status.HTTP_409_CONFLICT)
    if not file_path.is_file():
        return _not_found()
    return FileResponse(file_path, media_type='application/octet-stream', filename=file_path.name)


def _not_found() -> JSONResponse:
    """A 404 JSON response for a missing or unservable recording."""
    return JSONResponse(content={'status': 'error', 'message': 'Recording not found'},
                        status_code=status.HTTP_404_NOT_FOUND)


async def _browser_timezone() -> timezone:
    """The connected client's timezone, from its UTC offset; falls back to UTC.

    The offset comes from untrusted client JavaScript, so it is coerced to a number
    and clamped to a sane range; anything non-numeric, out of range, or slow to
    arrive falls back to UTC rather than breaking the page for that client.
    """
    try:
        await ui.context.client.connected()
        offset_minutes = await ui.run_javascript('new Date().getTimezoneOffset()')
        minutes = max(-_MAX_TIMEZONE_OFFSET_MINUTES, min(_MAX_TIMEZONE_OFFSET_MINUTES, float(offset_minutes)))
        return timezone(timedelta(minutes=-minutes))
    except (TimeoutError, TypeError, ValueError):
        return UTC


async def _confirm_dialog(message: str) -> bool:
    """Show a modal yes/no dialog and return whether the user confirmed.

    :param message: the question shown in the dialog.
    :return: ``True`` if the user confirmed, ``False`` if they cancelled or closed it.
    """
    with ui.dialog() as dialog, ui.card():
        ui.label(message)
        with ui.row():
            ui.button('Cancel', on_click=lambda: dialog.submit(False)).props('flat')
            ui.button('OK', on_click=lambda: dialog.submit(True))
    return bool(await dialog)


def _replace_sources(unfinished: Path, target: Path, sources: list[Path]) -> None:
    """Put the merged recording in place and drop the files it was made of.

    The rename happens before the deletion, so an interruption leaves the sources
    untouched rather than a gap.

    :param unfinished: the file the merge wrote.
    :param target: the name the merged recording takes.
    :param sources: the recordings the merge consumed.
    """
    unfinished.replace(target)
    for path in sources:
        path.unlink(missing_ok=True)
