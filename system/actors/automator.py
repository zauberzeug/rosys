from typing import Coroutine
from actors.actor import Actor
from world.state import State
from world.world import World


class Automator(Actor):

    def __init__(self, world: World):
        super().__init__(world)
        self.routines = []

    def add(self, coro: Coroutine):
        self.routines.append(coro)

    async def every_100_ms(self, world: World):

        if world.state != State.RUNNING:
            return

        for coro in self.routines:
            try:
                coro.send(None)
            except StopIteration:
                self.routines.remove(coro)
