from typing import Coroutine
from .actor import Actor
from ..world.state import State
from ..world.world import World


class Automator(Actor):

    interval: float = 0.1

    def __init__(self):
        self.routines = []

    def add(self, coro: Coroutine):
        self.routines.append(coro)

    async def step(self, world: World):

        if world.state != State.RUNNING:
            return

        for coro in self.routines:
            try:
                coro.send(None)
            except StopIteration:
                self.routines.remove(coro)
