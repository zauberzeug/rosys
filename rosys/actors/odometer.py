import numpy as np
from .actor import Actor
from ..world.world import World


class Odometer(Actor):

    def __init__(self):

        self.last_time: float = None
        self.state = Actor.State.active

    def update_pose(self, world: World):

        if self.last_time is None:
            self.last_time = world.time
            return

        dt = world.time - self.last_time

        robot = world.robot
        robot.pose.x += dt * robot.velocity.linear * np.cos(robot.pose.yaw)
        robot.pose.y += dt * robot.velocity.linear * np.sin(robot.pose.yaw)
        robot.pose.yaw += dt * robot.velocity.angular

        self.last_time = world.time
