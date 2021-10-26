import asyncio
from typing import Coroutine
from .actor import Actor
from ..world.world import World, WorldState


class Automator(Actor):
    interval: float = 0.1

    def __init__(self) -> None:
        super().__init__()
        self.routines = []

    def add(self, coro: Coroutine):
        self.routines.append(coro)

    def replace(self, coro: Coroutine):
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
            except:
                self.log.exception(f'paused and cleared automations due to exception in {coro}')
                world.state = WorldState.PAUSED
                self.routines.clear()
