from datetime import datetime
from pathlib import Path

from fastapi import HTTPException
from fastapi.responses import FileResponse
from nicegui import app, run, ui


class LogsPage:
    """Logs Page

    This module creates a page to list and download log files.
    It is mounted at /logs.
    """

    def __init__(self, *, logs_dir: Path | None = None) -> None:
        self.logs_dir = logs_dir or Path('~/.rosys').expanduser()

        @ui.page('/logs', title='Logs')
        async def page():
            await self._content()

        @app.get('/logs/{name:str}')
        async def download_log(name: str):
            path = self.logs_dir / name
            if not path.exists():
                raise HTTPException(status_code=404, detail=f'Log file {name} not found')
            return FileResponse(path)

    async def _content(self) -> None:
        @ui.refreshable
        def list_ui() -> None:
            with ui.card().tight().props('flat bordered'):
                if logs:
                    with ui.list():
                        for path in logs:
                            with ui.item(on_click=lambda path=path: ui.navigate.to(f'/logs/{path.name}', new_tab=True)):
                                with ui.item_section():
                                    ui.item_label(path.name)
                                    ui.item_label(_file_info(path)).props('caption')
                                with ui.item_section().props('side'):
                                    ui.button(icon='download').on('click.stop', lambda path=path: ui.download(path)) \
                                        .props('flat fab-mini').tooltip('download')
                else:
                    ui.label('No logs found')

        ui.label('Device Logs').classes('text-2xl')
        logs = await _find_log_files(self.logs_dir)
        list_ui()


async def _find_log_files(logs_dir: Path) -> list[Path]:
    logs = await run.io_bound(logs_dir.glob, '*.log')
    rotated = await run.io_bound(logs_dir.glob, '*.log.*')
    paths = list({p.resolve(): p for p in [*logs, *rotated]}.values())
    return sorted(paths, key=lambda p: p.stat().st_mtime, reverse=True)


def _file_info(path: Path) -> str:
    try:
        mtime = datetime.fromtimestamp(path.stat().st_mtime).strftime('%Y-%m-%d %H:%M:%S')
        size = _human_size(path.stat().st_size)
    except FileNotFoundError:
        mtime = 'n/a'
        size = 'n/a'
    return f'{mtime} • {size}'


def _human_size(num_bytes: int) -> str:
    units = ['B', 'KB', 'MB', 'GB', 'TB']
    size = float(num_bytes)
    unit = 0
    while size >= 1024 and unit < len(units) - 1:
        size /= 1024.0
        unit += 1
    return f'{size:.1f} {units[unit]}'
