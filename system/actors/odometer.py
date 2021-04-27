import numpy as np
from actors.actor import Actor
from world.world import World
from world.velocity import Velocity


class Odometer(Actor):

    last_time: float = None

    def update(self, velocity: Velocity, world: World):

        if self.last_time is None:
            self.last_time = world.time
            return

        dt = world.time - self.last_time

        robot = world.robot
        robot.pose.x += dt * velocity.linear * np.cos(robot.pose.yaw)
        robot.pose.y += dt * velocity.linear * np.sin(robot.pose.yaw)
        robot.pose.yaw += dt * velocity.angular

        self.last_time = world.time
