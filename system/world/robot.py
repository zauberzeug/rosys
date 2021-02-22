from pydantic import BaseModel
import numpy as np
from world.pose import Pose
from world.velocity import Velocity


class Robot(BaseModel):

    width: float
    pose: Pose = Pose()
    velocity: Velocity = Velocity()

    def drive(self, linear: float, angular: float):

        self.velocity.linear = linear
        self.velocity.angular = angular

    def power(self, left: float, right: float):

        self.velocity.linear = (left + right) / 2.0
        self.velocity.angular = (right - left) / self.width / 2.0

    def loop(self, dt: float):

        self.pose.x += dt * self.velocity.linear * np.cos(self.pose.yaw)
        self.pose.y += dt * self.velocity.linear * np.sin(self.pose.yaw)
        self.pose.yaw += dt * self.velocity.angular
