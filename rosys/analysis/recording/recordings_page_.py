from __future__ import annotations

from collections.abc import Callable
from datetime import UTC, date, datetime, timedelta, timezone
from pathlib import Path

from fastapi import status
from fastapi.responses import FileResponse, JSONResponse, Response
from nicegui import app, ui

from ... import rosys
from .mcap_recorder import McapRecorder, RecordingInfo
from .paths import DOWNLOAD_PATH, PAGE_PATH


class RecordingsPage:
    """Lists the MCAP recordings for download and deletion.

    The list refreshes whenever the recorder starts a new recording or stops one,
    can be filtered by date, and offers rebuilding the index of crash-orphaned
    (unindexed) recordings. All filesystem access (glob, stat, index check) runs
    off the event loop via ``rosys.run.io_bound``; the render reads only a cached
    snapshot, so opening the page never blocks the loop on disk I/O.

    A download endpoint at ``DOWNLOAD_PATH/{name}`` serves finished recordings over
    HTTP (basename only, refusing the live file with 409 and missing files with 404),
    so recordings can be fetched without scp.
    """

    def __init__(self, recorder: McapRecorder, *, header: Callable[[], None] | None = None) -> None:
        self.recorder = recorder
        self.header = header

        @ui.page(PAGE_PATH)
        async def page() -> None:
            if self.header is not None:
                self.header()
            await self.content()

        @app.get(f'{DOWNLOAD_PATH}/{{name}}')
        def download_recording(name: str) -> Response:
            file_path = recorder.output_dir / Path(name).name  # basename only, no path traversal
            if file_path == recorder.current_recording:
                return JSONResponse(content={'status': 'error', 'message': 'Recording is currently being written'},
                                    status_code=status.HTTP_409_CONFLICT)
            if not file_path.is_file():
                return JSONResponse(content={'status': 'error', 'message': 'Recording not found'},
                                    status_code=status.HTTP_404_NOT_FOUND)
            return FileResponse(file_path, media_type='application/octet-stream', filename=file_path.name)

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

        @ui.refreshable
        def recordings_list() -> None:
            reindex_button.set_enabled(any(not info.indexed and not info.is_live for info in infos))
            visible = [info for info in infos if _in_range(info)]
            if not visible:
                ui.label('No recordings.').classes('text-grey p-4')
                return
            with ui.column().classes('w-full gap-1'):
                for info in visible:
                    _recording_row(info)

        def _recording_row(info: RecordingInfo) -> None:
            modified = datetime.fromtimestamp(info.mtime, tz=local_tz).strftime('%Y-%m-%d %H:%M:%S')
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

        async def reload() -> None:
            infos[:] = await rosys.run.io_bound(recorder.scan_recordings) or []  # None on shutdown
            recordings_list.refresh()

        async def _delete(path: Path) -> None:
            if not await _confirm_dialog(f'Delete recording {path.name}?'):
                return
            recorder.delete_recording(path)
            await reload()

        async def _reindex() -> None:
            await recorder.reindex_unindexed()
            await reload()

        async def _delete_all() -> None:
            if not await _confirm_dialog('Delete all recordings?'):
                return
            recorder.delete_all_recordings()
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
                recorder.rename_recording(path, new_name)
                await reload()

        with ui.row().classes('w-full items-center gap-2'):
            date_input = ui.date_input('Date range', range_input=True, on_change=recordings_list.refresh)
            ui.space()
            reindex_button = ui.button(icon='build', on_click=_reindex) \
                .props('flat dense').tooltip('rebuild the index of unindexed recordings')
            ui.button(icon='delete_sweep', on_click=_delete_all) \
                .props('flat dense color=red').tooltip('delete all recordings')
            ui.button(icon='refresh', on_click=reload).props('flat dense')
        with ui.scroll_area().classes('w-full').style('max-height: 75vh'):
            recordings_list()

        async def sync_if_changed() -> None:
            # Picks up start/stop, size-based rotation, and external add/remove. The
            # change check itself is cheap and on-loop (a glob plus the current path);
            # the expensive stat/index scan in reload() runs off the loop. A
            # client-scoped timer (auto-removed on disconnect) avoids subscribing to
            # the process-lifetime recorder, so handlers don't accumulate across page
            # visits, and reloading only on an actual change keeps it off idle ticks.
            live = next((info.path for info in infos if info.is_live), None)
            if recorder.recordings != [info.path for info in infos] or recorder.current_recording != live:
                await reload()

        ui.timer(0.1, reload, once=True)  # populate the snapshot off the loop after the page is built
        ui.timer(2.0, sync_if_changed)


async def _browser_timezone() -> timezone:
    """The connected client's timezone, from its UTC offset; falls back to UTC."""
    try:
        await ui.context.client.connected()
        offset_minutes = await ui.run_javascript('new Date().getTimezoneOffset()')
        return timezone(timedelta(minutes=-offset_minutes))
    except TimeoutError:
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
