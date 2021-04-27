import numpy as np


class Odometer:

    def __init__(self):
        super().__init__()
        self.last_time: float = None

    def update_pose(self, world):

        if self.last_time is None:
            self.last_time = world.time
            return

        dt = world.time - self.last_time

        robot = world.robot
        robot.pose.x += dt * robot.velocity.linear * np.cos(robot.pose.yaw)
        robot.pose.y += dt * robot.velocity.linear * np.sin(robot.pose.yaw)
        robot.pose.yaw += dt * robot.velocity.angular

        self.last_time = world.time
