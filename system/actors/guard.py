import asyncio
from world.mode import Mode
from world.world import World
from actors.actor import Actor


class Guard(Actor):

    def __init__(self, world: World):
        self.world = world

    async def time_increment(self, seconds: float):
        if self.world.mode == Mode.TEST:
            end_time = self.world.time + seconds
            while self.world.time < end_time:
                await asyncio.sleep(0)
        else:
            await asyncio.sleep(seconds)

    async def condition(self, func):
        while not func(self.world.robot):
            await self.time_increment(0.05)
