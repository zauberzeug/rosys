import gc
import logging

import psutil

from .. import runtime


class GarbageCollector:
    '''Disabling Python's automatic garbage collection to optimize performance.'''
    interval: float = 10 * 60
    mbyte_limit: float = 300

    starting_msg = 'performing garbage collection'
    finished_msg = 'finished garbage collection'

    def __init__(self) -> None:
        self.log = logging.getLogger(self.__class__.__name__)
        gc.disable()

    async def step(self):
        await super().step()
        if psutil.virtual_memory().free < self.mbyte_limit * 1000000:
            self.log.warning(f'less than {self.mbyte_limit} mb of memory remaining -> {self.starting_msg}')
            gc.collect()
            await runtime.sleep(1)  # NOTE yield execution to ensure all warnings appear before sending "finish" message
            self.log.warning(self.finished_msg)
