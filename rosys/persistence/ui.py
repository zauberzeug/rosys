import json
from collections.abc import Callable
from pathlib import Path

from nicegui import app, events, ui
from starlette.responses import FileResponse

from ..helpers import invoke
from . import registry


def export_button(title: str = 'Export', route: str = '/export', tmp_filepath: Path = Path('/tmp/export.json')) -> ui.button:
    @app.get(route)
    def get_export() -> FileResponse:
        registry.write_export(tmp_filepath)
        return FileResponse(tmp_filepath, filename='export.json')
    return ui.button(title, on_click=lambda: ui.download(route[1:]))


def import_button(title: str = 'Import', after_import: Callable | None = None) -> ui.button:
    async def restore_from_file(e: events.UploadEventArguments) -> None:
        await registry.restore_from_export(json.load(e.content))
        dialog.close()
        if after_import is not None:
            await invoke(after_import)
    with ui.dialog() as dialog, ui.card():
        ui.upload(on_upload=restore_from_file)
    return ui.button(title, on_click=dialog.open)
