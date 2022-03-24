import gc

import psutil

from ..helpers import sleep
from .actor import Actor


class GarbageCollector(Actor):
    '''Disabling Python's automatic garbage collection to optimize performance.'''
    interval: float = 10 * 60
    mbyte_limit: float = 300

    starting_msg = 'performing garbage collection'
    finished_msg = 'finished garbage collection'

    def __init__(self) -> None:
        super().__init__()
        gc.disable()

    async def step(self):
        await super().step()
        if psutil.virtual_memory().free < self.mbyte_limit * 1000000:
            self.log.warning(f'less than {self.mbyte_limit} mb of memory remaining -> {self.starting_msg}')
            gc.collect()
            await sleep(1)  # NOTE yield execution to make sure all warnings appear before we send the "finish" message
            self.log.warning(self.finished_msg)
