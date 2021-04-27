from asyncio.locks import Event
from actors.actor import Actor
import asyncio
from world.mode import Mode
from world.world import World


class Automation(Actor):

    def __init__(self, world: World, allow_automation: Event):
        super().__init__(world)
        self.allow_automation = allow_automation

    async def can_proceed(self):
        await self.allow_automation.wait()
