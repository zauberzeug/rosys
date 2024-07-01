import datetime
import logging
from pathlib import Path

import rosys

from . import registry


class BackupSchedule:
    """The BackupSchedule module is responsible for backing up the persistence files every day at the specified time."""

    def __init__(self,
                 path: Path = Path('~/.rosys_backup'),
                 time: datetime.time = datetime.time(3, 0),
                 backup_count: int = 100) -> None:
        self.path = path.expanduser()
        self.path.mkdir(parents=True, exist_ok=True)
        self.time = time
        self.backup_count = backup_count
        self.log = logging.getLogger('rosys.persistence')

        rosys.on_repeat(self.backup, 60)

    def backup(self) -> None:
        """Backup the persistence files every day at the specified time."""
        now = datetime.datetime.now()
        filepath = self.path / f'{now:%Y-%m-%d}.json'
        if now.time() < self.time or filepath.exists():
            return
        self.log.info('Backing up persistence files to %s', filepath)
        registry.write_export(filepath)
        self.prune()

    def prune(self) -> None:
        """Delete old backups if there are more than the specified number."""
        while len(backups := sorted(self.path.glob('*.json'))) > self.backup_count:
            backups[0].unlink()
