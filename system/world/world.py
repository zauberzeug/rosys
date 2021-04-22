import asyncio
import sys
from pydantic import BaseModel
from world.robot import Robot


class World(BaseModel):

    robot: Robot

    async def run(self, seconds: float = sys.maxsize):

        end = self.robot.machine.time + seconds
        while self.robot.machine.time < end:
            await self.step()
            await asyncio.sleep(0)  # give other coroutines a chance to execute their code

    async def step(self):

        await self.robot.step()
