import numpy as np
from actors.actor import Actor


class Odometer(Actor):

    last_time: float = None

    async def step(self):

        if self.last_time is None:
            self.last_time = self.world.time
            return

        dt = self.world.time - self.last_time

        robot = self.world.robot
        robot.pose.x += dt * robot.velocity.linear * np.cos(robot.pose.yaw)
        robot.pose.y += dt * robot.velocity.linear * np.sin(robot.pose.yaw)
        robot.pose.yaw += dt * robot.velocity.angular

        self.last_time = self.world.time

        await self.sleep(0.01)
