from datetime import datetime
from pathlib import Path

from nicegui import run, ui


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

    async def _content(self) -> None:
        @ui.refreshable
        def list_ui() -> None:
            with ui.card().props('flat bordered'):
                if not logs:
                    ui.label('No logs found')
                    return
                with ui.list().classes('items-center'):
                    for p in logs:
                        title, caption = _format_log_entry(p)
                        with ui.item().classes('p-0'):
                            with ui.item_section().props('avatar'):
                                ui.button(icon='download', on_click=lambda p=p: ui.download(p)) \
                                    .props('flat').tooltip('download')
                            with ui.item_section().classes('min-w-24'):
                                ui.item_label(title)
                                ui.item_label(caption).props('caption')

        ui.label('Device Logs').classes('text-2xl')
        logs = await _list_log_files(self.logs_dir)
        list_ui()


def _human_size(num_bytes: int) -> str:
    units = ['B', 'KB', 'MB', 'GB', 'TB']
    size = float(num_bytes)
    unit = 0
    while size >= 1024 and unit < len(units) - 1:
        size /= 1024.0
        unit += 1
    return f'{size:.1f} {units[unit]}'


async def _list_log_files(logs_dir: Path) -> list[Path]:
    logs = await run.io_bound(logs_dir.glob, '*.log')
    rotated = await run.io_bound(logs_dir.glob, '*.log.*')
    paths = list({p.resolve(): p for p in [*logs, *rotated]}.values())

    def mtime(p: Path) -> float:
        try:
            return p.stat().st_mtime
        except FileNotFoundError:
            return 0.0
    return sorted(paths, key=mtime, reverse=True)


def _format_log_entry(path: Path) -> tuple[str, str]:
    try:
        size = _human_size(path.stat().st_size)
        mtime = datetime.fromtimestamp(path.stat().st_mtime).strftime('%Y-%m-%d %H:%M:%S')
    except FileNotFoundError:
        size = 'n/a'
        mtime = 'n/a'
    return path.name, f'{size} • {mtime}'
