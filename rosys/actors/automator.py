import asyncio
from typing import Coroutine
from .actor import Actor
from ..world.world import World, WorldState
import contextlib
import warnings
import gc


@contextlib.contextmanager
def silence_coro():
    '''Supress warning about coroutine is never awaited.

    This is ok because the Automator uses coro.send(None) to only step one chunck at a time.
    Similar code is used in  https://github.com/python/cpython/pull/5410'''
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        yield
        gc.collect()


class Automator(Actor):
    interval: float = 0.1

    def __init__(self) -> None:
        super().__init__()
        self.routines = []

    def add(self, coro: Coroutine):
        with silence_coro():
            self.routines.append(coro)

    def replace(self, coro: Coroutine):
        with silence_coro():
            self.routines.clear()
            self.add(coro)

    async def step(self, world: World):
        if world.state != WorldState.RUNNING:
            return

        for coro in self.routines:
            try:
                coro.send(None)
            except StopIteration:
                self.routines.remove(coro)
                if not self.routines:
                    await self.pause_automations(because='all have completed')
            except:
                await self.pause_automations(because='an exception occured in an automation')
                self.routines.clear()
                self.log.exception(f'paused and cleared automations due to exception in {coro}')
