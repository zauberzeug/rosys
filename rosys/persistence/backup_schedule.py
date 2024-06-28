import datetime
import logging
from pathlib import Path

import rosys

from . import registry


class BackupSchedule:

    def __init__(self, path: Path = Path('~/.rosysbackup'),
                 time: datetime.time = datetime.time(3, 0),
                 backup_count: int = 100) -> None:
        self.path = path.expanduser()
        self.time = time
        self.backup_count = backup_count
        self.log = logging.getLogger('rosys.persistence')

        if not self.path.exists():
            self.path.mkdir(parents=True)

        rosys.on_repeat(self.need_to_backup, 60)

    def need_to_backup(self) -> None:
        now = datetime.datetime.now()
        if now.time() < self.time or now.time() > self.time.replace(second=59):
            return
        backups = sorted(self.path.glob('*.json'))
        if len(backups) == self.backup_count:
            backups[0].unlink()
        self.log.info('Backing up persistence files to %s', self.path)
        registry.write_export(self.path / f'{now:%Y-%m-%d}.json')
