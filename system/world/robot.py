import asyncio
from pydantic import BaseModel
from world.pose import Pose
from world.velocity import Velocity
import task_logger
from typing import Coroutine


class Robot(BaseModel):

    pose: Pose = Pose()
    velocity: Velocity = Velocity()

    async def drive(self, linear: float, angular: float):

        await self.machine.send('drive speed %.3f,%.3f' % (linear, angular))

    async def power(self, left: float, right: float):

        await self.machine.send('drive pw %.3f,%.3f' % (left, right))

    def automate(_, coro: Coroutine):
        task_logger.create_task(coro)

    async def condition(self, func):
        while not func(self):
            await asyncio.sleep(0)
