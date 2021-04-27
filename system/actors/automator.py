from asyncio.locks import Event
from typing import Coroutine
from actors.actor import Actor
import asyncio
from world.mode import Mode
from world.world import World


class Automator(Actor):

    routines = []

    def __init__(self, world: World, allow_automation: Event):
        super().__init__(world)
        self.allow_automation = allow_automation

    async def can_proceed(self):
        await self.allow_automation.wait()

    def add(self, coro: Coroutine):
        self.routines.append(coro)

    async def every_100_ms(self):
        if not self.allow_automation.is_set():
            return

        for coro in self.routines:
            try:
                coro.send(None)
            except StopIteration:
                self.routines.remove(coro)
