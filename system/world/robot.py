import asyncio
from pydantic import BaseModel
from typing import Optional
import numpy as np
from world.pose import Pose
from world.machine import Machine
from world.velocity import Velocity
import task_logger
from typing import Coroutine


class Robot(BaseModel):

    machine: Optional[Machine]
    width: float
    pose: Pose = Pose()
    velocity: Velocity = Velocity()

    def drive(self, linear: float, angular: float):

        self.velocity.linear = linear
        self.velocity.angular = angular

    def power(self, left: float, right: float):

        if self.machine:
            self.machine.send('pw %.3f,%.3f' % (left, right))
        else:
            self.velocity.linear = (left + right) / 2.0
            self.velocity.angular = (right - left) / self.width / 2.0

    async def loop(self, dt: float):

        if self.machine:
            self.velocity = (await self.machine.read()) or self.velocity

        self.pose.x += dt * self.velocity.linear * np.cos(self.pose.yaw)
        self.pose.y += dt * self.velocity.linear * np.sin(self.pose.yaw)
        self.pose.yaw += dt * self.velocity.angular
        #print(self.pose, flush=True)

    def automate(_, coro: Coroutine):
        task_logger.create_task(coro)

    async def condition(self, func):
        while not func(self):
            await asyncio.sleep(0)
