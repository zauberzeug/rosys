import asyncio
from pydantic import BaseModel
from world.robot import Robot
from world.clock import Clock


class World(BaseModel):

    clock: Clock
    robot: Robot

    async def run(self):
        while True:
            self.loop()
            await asyncio.sleep(self.clock.interval)

    async def simulate(self, seconds):

        end = self.clock.time + seconds
        while self.clock.time < end:
            self.loop()
            await asyncio.sleep(0)  # give other coroutines a chance to execute their code

    def loop(self):

        self.clock.loop()
        self.robot.loop(self.clock.interval)
