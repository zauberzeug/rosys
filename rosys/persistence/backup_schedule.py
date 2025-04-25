import datetime
import logging
from pathlib import Path

from .. import rosys
from .import_export import export_all


class BackupSchedule:
    """The BackupSchedule module is responsible for backing up the persistence files every day at the specified time."""

    def __init__(self,
                 time: datetime.time | None = None,
                 path: Path = Path('~/.rosys_backup'),
                 backup_count: int = 100) -> None:
        """
        :param time: The time of day when the backup should be performed (default: 3:00).
        :param path: The directory where the backups should be stored (default: ~/.rosys_backup).
        :param backup_count: The number of backups to keep (default: 100).
        """
        self.path = path.expanduser()
        self.path.mkdir(parents=True, exist_ok=True)
        self.time = time or datetime.time(3, 0)
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
        export_all(filepath)
        self.prune()

    def prune(self) -> None:
        """Delete old backups if there are more than the specified number."""
        while len(backups := sorted(self.path.glob('*.json'))) > self.backup_count:
            backups[0].unlink()
