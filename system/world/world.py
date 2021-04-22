import asyncio
from pydantic import BaseModel
from world.robot import Robot
from world.clock import Clock


class World(BaseModel):

    clock: Clock
    robot: Robot

    async def run(self):
        while True:
            await self.loop()
            await asyncio.sleep(0)  # give other coroutines a chance to execute their code

    async def simulate(self, seconds):

        end = self.clock.time + seconds
        while self.clock.time < end:
            await self.loop()
            await asyncio.sleep(0)  # give other coroutines a chance to execute their code

    async def loop(self):

        self.clock.loop()
        await self.robot.loop(self.clock.interval)
