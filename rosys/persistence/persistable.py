import abc
import json
import re
from pathlib import Path
from typing import Any, ClassVar

from nicegui import app, run
from typing_extensions import Self

KEY_PATTERN = re.compile(r'^[a-zA-Z0-9_-]+$')


class Persistable(abc.ABC):
    used_keys: ClassVar[list[str]] = []

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._needs_backup = False
        self._filepath: Path | None = None

    def persistent(self, *,
                   key: str | None = None,
                   path: str | Path | None = None,
                   restore: bool = True,
                   backup_check_interval: float | None = 10.0,
                   backup_interval: float | None = None,
                   ) -> Self:
        """Make this object persistent.

        If ``restore`` is ``True``, the object will be restored from the persistence file.
        If ``backup_check_interval`` is not ``None`` or 0.0, the object will be backed up at the given interval _if a backup has been requested_.
        If ``backup_interval`` is not ``None`` or 0.0, the object will be backed up at the given interval _independently of any backup requests_.

        :param key: The key to use for the persistence file (letters, digits, dashes and underscores; default: module name)
        :param path: The path to the persistence file (default: "~/.rosys")
        :param restore: Whether to restore the object from the persistence file (default: True)
        :param backup_check_interval: The interval to check for backups (default: 10.0 seconds)
        :param backup_interval: The interval to backup the object (default: None)
        """
        if self._filepath is not None:
            raise RuntimeError('This object is already persistent')

        if key is None:
            key = self.__module__
        if not key:
            raise ValueError('Key is empty')
        if not KEY_PATTERN.match(key):
            raise ValueError(f'Key "{key}" contains invalid characters')
        if key in self.used_keys:
            raise ValueError(f'Key "{key}" is already used by another persistent object')
        self.used_keys.append(key)

        self._filepath = Path(path or '~/.rosys').expanduser() / f'{key}.json'
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
        """Request a backup of the object.

        This will trigger a backup at the check defined by ``backup_check_interval``.
        """
        self._needs_backup = True

    async def _backup_if_requested(self) -> None:
        if self._needs_backup:
            await self.backup()

    async def backup(self) -> None:
        """Backup the object.

        This will immediately write the object to the persistence file.
        The file is written in an I/O bound task to avoid blocking the main thread.
        """
        await run.io_bound(self.sync_backup)

    def sync_backup(self) -> None:
        """Backup the object.

        This will immediately write the object to the persistence file.
        The file is written synchronously.
        To avoid blocking the main thread, use ``request_backup()`` instead.
        """
        if self._filepath is None:
            raise RuntimeError('Backup failed: This object is not persistent.')
        self._filepath.write_text(json.dumps(self.backup_to_dict()))
        self._needs_backup = False

    async def restore(self) -> None:
        """Restore the object from the persistence file.

        The file is read in an I/O bound task to avoid blocking the main thread.
        """
        await run.io_bound(self.sync_restore)

    def sync_restore(self) -> None:
        """Restore the object from the persistence file.

        The file is read synchronously.
        To avoid blocking the main thread, use ``restore()`` instead.
        """
        if self._filepath is None:
            raise RuntimeError('Restore failed: This object is not persistent.')
        if not self._filepath.exists():
            raise FileNotFoundError(f'Restore failed: Persistence file "{self._filepath}" not found')
        self.restore_from_dict(json.loads(self._filepath.read_text()))

    def delete_persistence(self) -> None:
        """Delete the persistence file."""
        if self._filepath is None:
            raise RuntimeError('Delete persistence failed: This object is not persistent.')
        self._filepath.unlink()
        self._filepath = None

    @abc.abstractmethod
    def backup_to_dict(self) -> dict[str, Any]:
        """Convert the object to a dictionary."""

    @abc.abstractmethod
    def restore_from_dict(self, data: dict[str, Any]) -> None:
        """Restore the object from a dictionary."""
