import asyncio
import sys
from world.mode import Mode
from world.world import World


class Actor:

    def __init__(self, world: World):
        self.world = world

    async def condition(self, func):
        while not func():
            await self.can_proceed()
            await self.time_increment(0.05)

    async def time_increment(self, seconds: float, max_time: float = sys.maxsize):
        if self.world.mode == Mode.TEST:
            end_time = self.world.time + seconds
            while self.world.time < end_time and self.world.time < max_time:
                await self.can_proceed()
                await asyncio.sleep(0)
        else:
            await asyncio.sleep(seconds)

    async def can_proceed(self):
        pass
