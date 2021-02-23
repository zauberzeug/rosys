import asyncio
from utilities.angle import Angle, rad
from pydantic import BaseModel
import numpy as np
from world.pose import Pose
from world.velocity import Velocity


class Robot(BaseModel):

    width: float
    pose: Pose = Pose()
    velocity: Velocity = Velocity()

    def drive(self, linear: float, angular: Angle):
        self.velocity.linear = linear
        self.velocity.angular = angular

    def power(self, left: float, right: float):

        self.velocity.linear = (left + right) / 2.0
        self.velocity.angular = (right - left) / self.width

    def loop(self, dt: float):

        self.pose.x += dt * self.velocity.linear * np.cos(self.pose.yaw)
        self.pose.y += dt * self.velocity.linear * np.sin(self.pose.yaw)
        self.pose.yaw += rad(dt * self.velocity.angular)
        #print('pose', self.pose, flush=True)

    async def reaches(self, x: float, y: float, yaw: Angle = rad(0)):
        while abs(x - self.pose.x) > 0.01 or abs(y - self.pose.y) > 0.01 or abs(yaw - self.pose.yaw) > 0.001:
            await asyncio.sleep(0)

    async def condition(self, func):
        while not func(self):
            await asyncio.sleep(0)
