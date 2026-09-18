from dataclasses import dataclass
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
                        for log in logs:
                            with ui.item(on_click=lambda log=log: ui.navigate.to(f'/logs/{log.path.name}', new_tab=True)):
                                with ui.item_section():
                                    ui.item_label(log.path.name)
                                    ui.item_label(_file_info(log)).props('caption')
                                with ui.item_section().props('side'):
                                    ui.button(icon='download').on('click.stop', lambda log=log: ui.download(log.path)) \
                                        .props('flat fab-mini').tooltip('download')
                else:
                    ui.label('No logs found')

        ui.label('Device Logs').classes('text-2xl')
        logs = await _find_log_files(self.logs_dir)
        list_ui()


@dataclass(slots=True, kw_only=True)
class _LogFile:
    path: Path
    mtime: float
    size: int


async def _find_log_files(logs_dir: Path) -> list[_LogFile]:
    def scan() -> list[_LogFile]:
        paths = {p.resolve(): p for p in [*logs_dir.glob('*.log'), *logs_dir.glob('*.log.*')]}
        logs: list[_LogFile] = []
        for path in paths.values():
            try:
                stat = path.stat()
            except OSError:
                continue
            logs.append(_LogFile(path=path, mtime=stat.st_mtime, size=stat.st_size))
        return sorted(logs, key=lambda log: log.mtime, reverse=True)
    return await run.io_bound(scan) or []


def _file_info(log: _LogFile) -> str:
    mtime = datetime.fromtimestamp(log.mtime).strftime('%Y-%m-%d %H:%M:%S')
    return f'{mtime} • {_human_size(log.size)}'


def _human_size(num_bytes: int) -> str:
    units = ['B', 'KB', 'MB', 'GB', 'TB']
    size = float(num_bytes)
    unit = 0
    while size >= 1024 and unit < len(units) - 1:
        size /= 1024.0
        unit += 1
    return f'{size:.1f} {units[unit]}'
