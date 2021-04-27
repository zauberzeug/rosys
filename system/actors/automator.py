from asyncio.locks import Event
from typing import Coroutine
from world.state import State
from actors.actor import Actor
import asyncio
from world.mode import Mode
from world.world import World


class Automator(Actor):

    routines = []

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
