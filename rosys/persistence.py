import json
import logging
import sys
from dataclasses import fields
from pathlib import Path
from typing import Any, Callable, Optional, Protocol

import numpy as np
from dataclasses_json import Exclude, config
from dataclasses_json.core import _asdict, _decode_dataclass
from nicegui import app, ui
from nicegui.events import UploadEventArguments
from starlette.responses import FileResponse

from .helpers import invoke
from .run import awaitable

exclude: dict[str, dict] = config(exclude=Exclude.ALWAYS)


def to_dict(obj: Any) -> dict[str, Any]:
    return _asdict(obj, False)


def from_dict(cls: type, d: dict[str, Any]) -> Any:
    return _decode_dataclass(cls, d, False)


def replace_dict(old_dict: dict[str, Any], cls: type, new_dict: dict[str, Any]) -> None:
    """Replace content of `old_dict` with keys and values from `new_dict`."""
    old_dict.clear()
    old_dict.update({key: from_dict(cls, value) for key, value in new_dict.items()})


def replace_list(old_list: list[Any], cls: type, new_list: list[Any]) -> None:
    """Replace content of `old_list` with items from `new_list`."""
    old_list.clear()
    old_list.extend(from_dict(cls, value) for value in new_list)


def replace_dataclass(old_dataclass: Any, new_dict: dict[str, Any]) -> None:
    """Replace content of `old_dataclass` with content from `new_dict`."""
    for field in fields(old_dataclass):
        setattr(old_dataclass, field.name, new_dict.get(field.name))


backup_path = Path('~/.rosys').expanduser()
log = logging.getLogger('rosys.persistence')

is_test = 'pytest' in sys.modules


class PersistentModule(Protocol):
    needs_backup: bool

    def backup(self) -> dict[str, Any]:
        ...

    def restore(self, data: dict[str, Any]) -> None:
        ...


modules: dict[str, PersistentModule] = {}


def register(module: PersistentModule) -> None:
    if not is_test:
        modules[module.__module__] = module


class Encoder(json.JSONEncoder):

    def default(self, o):
        if isinstance(o, np.floating):
            return float(o)
        return json.JSONEncoder.default(self, o)


@awaitable
def backup(force: bool = False) -> None:
    for name, module in modules.items():
        if not module.needs_backup and not force:
            continue
        if not backup_path.exists():
            backup_path.mkdir(parents=True)
        filepath = backup_path / f'{name}.json'
        filepath.write_text(json.dumps(module.backup(), indent=4, cls=Encoder))
        module.needs_backup = False


def restore() -> None:
    for name, module in modules.items():
        filepath = backup_path / f'{name}.json'
        if not filepath.exists():
            log.warning(f'Backup file "{filepath}" not found.')
            continue
        try:
            module.restore(json.loads(filepath.read_text()))
        except Exception:
            log.exception(f'failed to restore {module}')


def export_button(title: str = 'Export', route: str = '/export', tmp_filepath: Path = Path('/tmp/export.json')) -> ui.button:
    @app.get(route)
    def get_export() -> FileResponse:
        data = {name: module.backup() for name, module in modules.items()}
        tmp_filepath.write_text(json.dumps(data, indent=4))
        return FileResponse(tmp_filepath, filename='export.json')
    return ui.button(title, on_click=lambda: ui.download(route[1:]))


def import_button(title: str = 'Import', after_import: Optional[Callable] = None) -> ui.button:
    async def restore_from_file(e: UploadEventArguments) -> None:
        all_data = json.load(e.content)
        assert isinstance(all_data, dict)
        for name, data in all_data.items():
            modules[name].restore(data)
        await backup(force=True)
        dialog.close()
        if after_import is not None:
            await invoke(after_import)
    with ui.dialog() as dialog, ui.card():
        ui.upload(on_upload=restore_from_file)
    return ui.button(title, on_click=dialog.open)
