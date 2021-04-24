import sys
import asyncio
from world.mode import Mode
from world.world import World


class Actor:

    def __init__(self, world: World):

        self.world = world

    async def run(self, end_time: float = sys.maxsize):

        self.run_end_time = end_time
        while self.world.time < end_time:
            await self.step()

    async def sleep(self, seconds: float):

        sleep_end_time = min(self.world.time + seconds, self.run_end_time)
        if self.world.mode == Mode.TEST:
            while self.world.time <= sleep_end_time:
                await asyncio.sleep(0)
        else:
            await asyncio.sleep(sleep_end_time - self.world.time)
