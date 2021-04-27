import asyncio
import sys
from world.mode import Mode
from world.world import World


class Actor:

    def __init__(self, world: World):
        self.world = world

    async def sleep(self, seconds: float, max_time: float = sys.maxsize):
        if self.world.mode == Mode.TEST:
            end_time = self.world.time + seconds
            while self.world.time < end_time and self.world.time < max_time:
                await asyncio.sleep(0)
        else:
            await asyncio.sleep(seconds)
