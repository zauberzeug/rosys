from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from ..helpers import is_test
from ..run import awaitable

if TYPE_CHECKING:
    from .persistent_module import PersistentModule

log = logging.getLogger('rosys.persistence')
backup_path = Path('~/.rosys').expanduser()
modules: dict[str, PersistentModule] = {}


def register(module: PersistentModule, key: str | None = None) -> None:
    if not is_test():
        modules[key or module.__module__] = module


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
        try:
            temp_filepath = filepath.with_suffix('.tmp')
            temp_filepath.write_text(json.dumps(module.backup(), indent=4, cls=Encoder))
            temp_filepath.rename(filepath)
        except Exception:
            log.exception('failed to backup %s: %s', module, str(module.backup()))
            if temp_filepath.exists():
                temp_filepath.unlink()
        module.needs_backup = False


def restore() -> None:
    for name, module in modules.items():
        filepath = backup_path / f'{name}.json'
        if not filepath.exists():
            log.warning('Backup file "%s" not found.', filepath)
            continue
        file_content = filepath.read_text()
        try:
            module.restore(json.loads(file_content))
        except Exception:
            log.exception('failed to restore %s', module)


def write_export(to_filepath: Path) -> None:
    data = {name: module.backup() for name, module in modules.items()}
    to_filepath.write_text(json.dumps(data, indent=4))
