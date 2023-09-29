import json
from pathlib import Path
from typing import Callable, Optional

from nicegui import app, events, ui
from starlette.responses import FileResponse

from ..helpers import invoke
from . import registry


def export_button(title: str = 'Export', route: str = '/export', tmp_filepath: Path = Path('/tmp/export.json')) -> ui.button:
    @app.get(route)
    def get_export() -> FileResponse:
        data = {name: module.backup() for name, module in registry.modules.items()}
        tmp_filepath.write_text(json.dumps(data, indent=4))
        return FileResponse(tmp_filepath, filename='export.json')
    return ui.button(title, on_click=lambda: ui.download(route[1:]))


def import_button(title: str = 'Import', after_import: Optional[Callable] = None) -> ui.button:
    async def restore_from_file(e: events.UploadEventArguments) -> None:
        all_data = json.load(e.content)
        assert isinstance(all_data, dict)
        for name, data in all_data.items():
            registry.modules[name].restore(data)
        await registry.backup(force=True)
        dialog.close()
        if after_import is not None:
            await invoke(after_import)
    with ui.dialog() as dialog, ui.card():
        ui.upload(on_upload=restore_from_file)
    return ui.button(title, on_click=dialog.open)
