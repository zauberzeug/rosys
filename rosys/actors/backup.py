import logging

from .. import run


class Backup:
    interval: float = 10

    def __init__(self) -> None:
        self.log = logging.getLogger('rosys.backup')

    async def step(self):
        if self.world.needs_backup:
            self.log.info('saving the world')
            await run.io_bound(self.persistance.backup)
            self.world.needs_backup = False
