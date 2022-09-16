import json
import logging
import os
import sys
from dataclasses import fields
from typing import Any, Callable, Optional, Protocol, TypeVar

import numpy as np
from dataclasses_json import Exclude, config
from dataclasses_json.core import _asdict, _decode_dataclass
from nicegui import ui
from nicegui.events import UploadEventArguments
from starlette.responses import FileResponse

from .helpers import invoke
from .run import awaitable

exclude = config(exclude=Exclude.ALWAYS)


def to_dict(obj: Any) -> dict[str, Any]:
    return _asdict(obj, False)


def from_dict(cls: type, d: dict[str, Any]) -> Any:
    return _decode_dataclass(cls, d, False)


T = TypeVar('T')


def replace_dict(old_dict: dict[str, T], cls: type, new_dict: dict[str, T]) -> None:
    '''Replace content of `old_dict` with keys and values from `new_dict`.'''
    old_dict.clear()
    old_dict.update({key: from_dict(cls, value) for key, value in new_dict.items()})


def replace_list(old_list: list[T], cls: type, new_list: list[T]) -> None:
    '''Replace content of `old_list` with items from `new_list`.'''
    old_list.clear()
    old_list.extend(from_dict(cls, value) for value in new_list)


def replace_dataclass(old_dataclass: Any, new_dict: dict[str, Any]) -> None:
    '''Replace content of `old_dataclass` with content from `new_dict`.'''
    for field in fields(old_dataclass):
        setattr(old_dataclass, field.name, new_dict.get(field.name))


backup_path = os.path.expanduser('~/.rosys')
log = logging.getLogger('rosys.persistence')

is_test = 'pytest' in sys.modules


class PersistentActor(Protocol):
    needs_backup: bool

    def backup(self) -> dict[str, Any]:
        ...

    def restore(self, data: dict[str, Any]) -> None:
        ...


actors: dict[str, PersistentActor] = {}


def register(actor: PersistentActor) -> None:
    if not is_test:
        actors[actor.__module__] = actor


class Encoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.floating):
            return float(obj)
        return json.JSONEncoder.default(self, obj)


@awaitable
def backup(force: bool = False) -> None:
    for name, actor in actors.items():
        if not actor.needs_backup and not force:
            continue
        if not os.path.exists(backup_path):
            os.makedirs(backup_path)
        filepath = f'{backup_path}/{name}.json'
        with open(filepath, 'w') as f:
            json.dump(actor.backup(), f, indent=4, cls=Encoder)
        actor.needs_backup = False


def restore() -> None:
    for name, actor in actors.items():
        filepath = f'{backup_path}/{name}.json'
        if not os.path.exists(filepath):
            log.warning(f'Backup file "{filepath}" not found.')
            continue
        with open(filepath) as f:
            actor.restore(json.load(f))


def export_button(title: str = 'Export', route: str = '/export', tmp_filepath: str = '/tmp/export.json') -> ui.button:
    @ui.get(route)
    def get_export() -> FileResponse:
        with open(tmp_filepath, 'w') as f:
            json.dump({name: actor.backup() for name, actor in actors.items()}, f, indent=4)
        return FileResponse(tmp_filepath, filename='export.json')
    ui.button(title, on_click=lambda e: ui.open(route[1:], e.socket))


def import_button(title: str = 'Import', after_import: Optional[Callable] = None) -> ui.button:
    async def restore_from_file(e: UploadEventArguments) -> None:
        all_data = json.loads(e.files[0].decode())
        assert isinstance(all_data, dict)
        for name, data in all_data.items():
            actors[name].restore(data)
        await backup(force=True)
        dialog.close()
        await invoke(after_import)
    with ui.dialog() as dialog, ui.card():
        ui.upload(on_upload=restore_from_file, upload_button_text='Import')
    return ui.button(title, on_click=dialog.open)
