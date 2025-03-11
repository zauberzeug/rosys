import abc
import json
from pathlib import Path
from typing import Any, ClassVar, Self
from nicegui import app, run


class Persistable(abc.ABC):
    used_keys: ClassVar[list[str]] = []

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._needs_backup = False
        self._filepath: Path | None = None

    def persistent(self, *,
                   key: str | None = None,
                   path: str | Path = Path('~/.rosys').expanduser(),
                   restore: bool = True,  # TODO: which default?
                   backup_check_interval: float | None = 10.0,  # TODO: which default?
                   backup_interval: float | None = None,  # TODO: which default?
                   ) -> Self:
        if self._filepath is not None:
            raise RuntimeError('This object is already persistent')
        key = key or self.__module__
        if key in self.used_keys:
            raise ValueError(f'Key "{key}" is already used by another persistent object')
        self.used_keys.append(key)
        self._filepath = Path(path) / f'{key}.json'
        if restore:
            self.sync_restore()
        if backup_check_interval:
            app.timer(backup_check_interval, self._backup_if_requested)
        if backup_interval:
            app.timer(backup_interval, self.backup)
        if backup_check_interval or backup_interval:
            app.on_shutdown(self.sync_backup)  # TODO: only if requested?
        return self

    def request_backup(self) -> None:
        self._needs_backup = True

    async def _backup_if_requested(self) -> None:
        if self._needs_backup:
            await self.backup()

    async def backup(self) -> None:
        await run.io_bound(self.sync_backup)

    def sync_backup(self) -> None:
        if self._filepath is None:
            raise RuntimeError('Backup failed: This object is not persistent.')
        self._filepath.write_text(json.dumps(self.backup_to_dict()))
        self._needs_backup = False

    async def restore(self) -> None:
        await run.io_bound(self.sync_restore)

    def sync_restore(self) -> None:
        if self._filepath is None:
            raise RuntimeError('Restore failed: This object is not persistent.')
        if not self._filepath.exists():
            raise FileNotFoundError(f'Restore failed: Persistence file "{self._filepath}" not found')
        self.restore_from_dict(json.loads(self._filepath.read_text()))

    def delete_persistence(self) -> None:
        if self._filepath is None:
            raise RuntimeError('Delete persistence failed: This object is not persistent.')
        self._filepath.unlink()
        self._filepath = None

    @abc.abstractmethod
    def backup_to_dict(self) -> dict[str, Any]:
        pass

    @abc.abstractmethod
    def restore_from_dict(self, data: dict[str, Any]) -> None:
        pass
