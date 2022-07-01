import logging

from .. import run
from ..persistence import Persistence


class Backup:
    interval: float = 10

    def __init__(self, persistance: Persistence) -> None:
        self.log = logging.getLogger('rosys.backup')
        self.persistance = persistance

    async def step(self):
        if self.world.needs_backup:
            self.log.info('saving the world')
            await run.io_bound(self.persistance.backup)
            self.world.needs_backup = False
