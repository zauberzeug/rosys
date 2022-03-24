from .. import run
from ..persistence import Persistence
from .actor import Actor


class Backup(Actor):
    interval: float = 10

    def __init__(self, persistance: Persistence) -> None:
        super().__init__()
        self.persistance = persistance

    async def step(self):
        await super().step()
        if self.world.needs_backup:
            self.log.info('saving the world')
            await run.io_bound(self.persistance.backup)
            self.world.needs_backup = False
