from utilities.angle import Angle
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

    def loop(self, dt: float):

        self.pose.x += dt * self.velocity.linear * np.cos(self.pose.yaw)
        self.pose.y += dt * self.velocity.linear * np.sin(self.pose.yaw)
        self.pose.yaw += dt * self.velocity.angular
