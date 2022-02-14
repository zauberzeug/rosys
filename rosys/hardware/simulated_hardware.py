import numpy as np
from ..world import Velocity, World
from .hardware import Hardware


class SimulatedHardware(Hardware):

    def __init__(self, world: World):
        super().__init__(world)
        self.linear_velocity: float = 0
        self.angular_velocity: float = 0

    async def configure(self):
        raise Exception('there is no communication for configuring hardware')

    async def restart(self):
        raise Exception('there is no communication for restarting hardware')

    async def drive(self, linear: float, angular: float):
        self.linear_velocity = linear
        self.angular_velocity = angular

    async def stop(self):
        self.linear_velocity = 0
        self.angular_velocity = 0

    async def update(self):
        self.world.robot.odometry.append(Velocity(
            linear=self.linear_velocity,
            angular=self.angular_velocity,
            time=self.world.time,
        ))
        self.world.robot.battery = 25.0 + np.sin(0.1 * self.world.time) + 0.02 * np.random.randn()
        self.world.robot.temperature = np.random.uniform(34, 35)
