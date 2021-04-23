import asyncio
from pydantic import BaseModel, PrivateAttr
from typing import Optional
import numpy as np
from world.pose import Pose
from world.machine import Machine
from world.velocity import Velocity
import task_logger
from typing import Coroutine


class Robot(BaseModel):

    machine: Machine
    pose: Pose = Pose()
    velocity: Velocity = Velocity()
    _last_read: Optional[float] = PrivateAttr(None)

    async def drive(self, linear: float, angular: float):

        await self.machine.send('drive speed %.3f,%.3f' % (linear, angular))

    async def power(self, left: float, right: float):

        await self.machine.send('drive pw %.3f,%.3f' % (left, right))

    async def step(self):

        try:
            self.velocity = await self.machine.read()
            if self._last_read is None:
                self._last_read = self.machine.time
                return
            dt = self.machine.time - self._last_read
            self._last_read = self.machine.time
        except IOError as e:
            print(e)
            self.velocity = Velocity(linear=0, angular=0)
            dt = 1.0  # NOTE: does not matter
        self.pose.x += dt * self.velocity.linear * np.cos(self.pose.yaw)
        self.pose.y += dt * self.velocity.linear * np.sin(self.pose.yaw)
        self.pose.yaw += dt * self.velocity.angular
        #print(self.pose, flush=True)

    def automate(_, coro: Coroutine):
        task_logger.create_task(coro)

    async def condition(self, func):
        while not func(self):
            await asyncio.sleep(0)
