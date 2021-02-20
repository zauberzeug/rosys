from pydantic import BaseModel
import numpy as np
from world.pose import Pose
from world.velocity import Velocity


class Robot(BaseModel):

    width: float
    pose: Pose = Pose()
    velocity: Velocity = Velocity()

    def drive(self, m_per_s: float, rad_per_s: float):

        self.velocity.linear = m_per_s
        self.velocity.angular = rad_per_s

    def loop(self, dt: float):

        self.pose.x += dt * self.velocity.linear * np.cos(self.pose.yaw)
        self.pose.y += dt * self.velocity.linear * np.sin(self.pose.yaw)
        self.pose.yaw += dt * self.velocity.angular
