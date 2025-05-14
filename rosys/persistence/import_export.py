import json
from collections.abc import Callable
from pathlib import Path
from typing import Any

from nicegui import events, ui

from .. import helpers
from .persistable import Persistable


def export_all(path: Path | None = None, *, indent: int = 4) -> dict[str, dict[str, Any]]:
    """Export all persistable objects to a dictionary or file."""
    data = {key: instance.backup_to_dict() for key, instance in Persistable.instances.items()}
    if path:
        path.write_text(json.dumps(data, indent=indent))
    return data


def import_all(source: Path | dict[str, dict[str, Any]]) -> None:
    """Import all persistable objects from a file or dictionary.

    Note that objects that are not in the source dictionary are silently skipped.
    Exceptions that occur while restoring an individual object are not caught.
    """
    if isinstance(source, Path):
        data = json.loads(source.read_text())
    else:
        data = source
    for key, instance in Persistable.instances.items():
        if key in data:
            instance.restore_from_dict(data[key])


def export_button(title: str = 'Export', *,
                  icon: str | None = None,
                  color: str | None = 'primary',
                  indent: int = 4) -> ui.button:
    return ui.button(title, icon=icon, color=color,
                     on_click=lambda: ui.download.content(json.dumps(export_all(), indent=indent),
                                                          filename='export.json'))


def import_button(title: str = 'Import', *,
                  icon: str | None = None,
                  color: str | None = 'primary',
                  on_completion: Callable | None = None) -> ui.button:
    async def restore_from_file(e: events.UploadEventArguments) -> None:
        import_all(json.load(e.content))
        dialog.close()
        if on_completion is not None:
            await helpers.invoke(on_completion)
    with ui.dialog() as dialog:
        ui.upload(label=title, on_upload=restore_from_file).props('flat')
    return ui.button(title, icon=icon, color=color, on_click=dialog.open)
