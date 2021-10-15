import asyncio
import logging
from typing import Callable


class Actor:
    interval: float = None

    def __init__(self) -> None:
        name = __name__[:-5] + self.__class__.__name__
        self.log = logging.getLogger(name)

    async def step(self):
        pass

    async def tear_down(self):
        pass

    async def sleep(self, seconds: float):
        '''delay execution; in tests this method will be replaced'''
        await asyncio.sleep(seconds)

    async def run_in_executor(self, callback: Callable, *args: any):
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, callback, *args)

    def __str__(self) -> str:
        return type(self).__name__
