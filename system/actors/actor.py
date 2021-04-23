import sys
import asyncio
from world.world import World
from icecream import ic


class Actor:

    def __init__(self, world: World):

        self.world = world

    async def run(self, seconds: float = sys.maxsize):

        end = self.world.time + seconds
        while self.world.time < end:
            await self.step()

    async def sleep(self, seconds: float):

        end = self.world.time + seconds
        while self.world.time < end:
            ic(self, self.world.time)
            await asyncio.sleep(0)
