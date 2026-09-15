from __future__ import annotations

import asyncio
from collections.abc import Callable
from datetime import UTC, date, datetime, timedelta, timezone
from pathlib import Path

from fastapi import status
from fastapi.responses import FileResponse, JSONResponse, Response
from nicegui import app, ui

from ... import rosys
from .mcap_recorder import _OWN_NAME, McapRecorder, RecordingInfo
from .paths import DOWNLOAD_PATH, PAGE_PATH

_RIGHT_INSET = 'var(--nicegui-default-padding)'  # clears Quasar's 10 px thumb and keeps the page's rhythm

_MAX_TIMEZONE_OFFSET_MINUTES = 24 * 60  # reject offsets beyond ±24 h from untrusted client JavaScript


class RecordingsPage:
    """Lists the MCAP recordings for download and deletion.

    A long recording rotates into many files, so the parts of one run are listed as a
    single entry that expands into its parts and can be merged into one recording; any
    other file stays a row of its own. The list refreshes whenever the recorder starts a
    new recording or stops one, can be filtered by date, and offers rebuilding the index
    of crash-orphaned (unindexed) recordings. All filesystem access (glob, stat, index
    check) runs off the event loop via ``rosys.run.io_bound``; the render reads only a
    cached snapshot, so opening the page never blocks the loop on disk I/O.

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
        shown_merges: set[str] = set()  # merges in progress as last rendered, so the sync notices another client's

        @ui.refreshable
        def recordings_list() -> None:
            reindex_button.set_enabled(any(not info.indexed and not info.is_live for info in infos))
            shown_merges.clear()
            shown_merges.update(recorder.merging)
            entries = [(run, parts) for run, parts in _group_by_run(infos) if any(_in_range(part) for part in parts)]
            if not entries:
                ui.label('No recordings.').classes('text-grey p-4')
                return
            with ui.column().classes('w-full gap-1'):
                for run, parts in entries:
                    if run is None or len(parts) == 1:
                        _recording_row(parts[0])
                    else:
                        _run_row(run, parts)

        def _run_row(run: str, parts: list[RecordingInfo]) -> None:
            """Render the files of one run as one entry that expands into its parts.

            The entry keeps the shape of a single recording's row, so a run and a lone
            recording sit on the same edges; only the parts inside are indented.

            :param run: the name the run's files share.
            :param parts: the run's files, newest first.
            """
            first = datetime.fromtimestamp(min(part.mtime for part in parts), tz=local_tz)
            last = datetime.fromtimestamp(max(part.mtime for part in parts), tz=local_tz)
            size = sum(part.size for part in parts)
            is_expanded = run in expanded_runs

            def toggle() -> None:
                expanded_runs.symmetric_difference_update({run})
                recordings_list.refresh()

            with ui.row().classes('w-full items-center justify-between border-b py-1'):
                with ui.column().classes('gap-0'):
                    with ui.row().classes('items-center gap-1'):
                        ui.label(run).classes('font-mono')
                        if any(part.is_live for part in parts):
                            ui.badge('recording').props('color=red')
                    ui.label(f'{first:%Y-%m-%d %H:%M:%S} - {last:%H:%M:%S} · '
                             f'{size / 1_048_576:.1f} MB · {len(parts)} parts').classes('text-xs text-grey')
                with ui.row().classes('gap-1'):
                    if _merged_name(run) in recorder.merging:
                        ui.spinner(size='sm').tooltip('merging this run into one recording')
                    elif not any(part.is_live for part in parts):
                        ui.button(icon='merge', on_click=lambda r=run, p=list(parts): _merge_run(r, p)) \
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

        async def _merge_run(run: str, parts: list[RecordingInfo]) -> None:
            """Merge a run in the background, so closing the page does not cancel it.

            :param run: the run whose files are merged.
            :param parts: the run's files, newest first.
            """
            rosys.background_tasks.create(_merge(run, [part.path for part in reversed(parts)]), name=f'merge {run}')
            await asyncio.sleep(0)  # lets the merge claim its name, so the list shows it in progress
            recordings_list.refresh()

        async def _merge(run: str, sources: list[Path]) -> None:
            """Merge the run's files and tell how it went.

            :param run: the run whose files are merged.
            :param sources: the run's files, oldest first.
            """
            try:
                target = await recorder.merge(sources, _merged_name(run))
            except Exception as e:
                rosys.notify(f'Could not merge {run}: {e}', type='negative')
            else:
                if target is None:
                    rosys.notify(f'Nothing to merge in {run}', type='warning')
                else:
                    rosys.notify(f'Merged {len(sources)} files into {target.name}', type='positive')
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
                try:
                    if await rosys.run.io_bound(recorder.rename_recording, path, new_name) is None:  # off the loop
                        raise ValueError('name already in use or invalid')
                except ValueError as e:
                    rosys.notify(f'Could not rename {path.name}: {e}', type='negative')
                await reload()

        with ui.column().classes('w-full gap-2'):
            with ui.row().classes('w-full items-center gap-2').style(f'padding-right: {_RIGHT_INSET}'):
                date_input = ui.date_input('Date range', range_input=True, on_change=recordings_list.refresh)
                ui.space()
                reindex_button = ui.button(icon='build', on_click=_reindex) \
                    .props('flat dense').tooltip('rebuild the index of unindexed recordings')
                ui.button(icon='delete_sweep', on_click=_delete_all) \
                    .props('flat dense color=red').tooltip('delete all recordings')
                ui.button(icon='refresh', on_click=reload).props('flat dense')
            # the page already insets its content; the scroll area would add a second one, only for the list
            # Quasar swaps in the active style once a thumb shows, so both carry the same padding
            content_padding = f'padding-left: 0; padding-right: {_RIGHT_INSET}'
            with ui.scroll_area().classes('w-full') \
                    .props(f'content-style="{content_padding}" content-active-style="{content_padding}"') \
                    .style('max-height: 75vh'):
                recordings_list()

        def _change_token() -> tuple[tuple[Path, ...], Path | None, frozenset[str]]:
            """A cheap change token — the recording paths, the live file and the merges in progress.

            Both accessors glob the output directory, so this runs off the event loop.
            """
            return tuple(recorder.recordings), recorder.current_recording, recorder.merging

        async def sync_if_changed() -> None:
            # Picks up start/stop, rotation, merges and external add/remove. The
            # change token globs the directory, so it is computed off the loop (matching
            # the class docstring); only an actual change triggers the fuller stat/index
            # scan in reload(). A client-scoped timer (auto-removed on disconnect) avoids
            # subscribing to the process-lifetime recorder, so handlers don't accumulate
            # across page visits, and reloading only on a change keeps it off idle ticks.
            token = await rosys.run.io_bound(_change_token)
            if token is None:
                return  # recorder shut down mid-scan
            live = next((info.path for info in infos if info.is_live), None)
            if token != (tuple(info.path for info in infos), live, frozenset(shown_merges)):
                await reload()

        ui.timer(0.1, reload, once=True)  # populate the snapshot off the loop after the page is built
        ui.timer(2.0, sync_if_changed)


def _group_by_run(infos: list[RecordingInfo]) -> list[tuple[str | None, list[RecordingInfo]]]:
    """Group the files that belong to the same run, keeping the order of their newest file.

    A run's files are the parts the recorder numbered apart while rotating; every other
    file (a renamed, merged or preserved recording) forms an entry of its own.

    :param infos: the recordings to group, in display order.
    :return: one entry per run with its parts, the highest part number first; ``None`` as the
        run for a file that belongs to no run.
    """
    entries: list[tuple[str | None, list[RecordingInfo]]] = []
    runs: dict[str, list[tuple[int, RecordingInfo]]] = {}
    for info in infos:
        match = _OWN_NAME.fullmatch(info.path.name)
        if match is None or match.group('run') is None:
            entries.append((None, [info]))
            continue
        run = match.group('run')
        if run not in runs:
            runs[run] = []
            entries.append((run, []))
        runs[run].append((int(match.group('part')), info))
    for run, parts in entries:
        if run is not None:
            parts.extend(info for _, info in sorted(runs[run], key=lambda numbered: numbered[0], reverse=True))
    return entries


def _merged_name(run: str) -> str:
    """The name a run takes once merged, which never reads as one of the recorder's own files."""
    return f'{run}_merged'


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
