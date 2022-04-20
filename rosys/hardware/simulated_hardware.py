from ..world import Velocity, World
from .hardware import Hardware


class SimulatedHardware(Hardware):

    def __init__(self, world: World):
        super().__init__(world)
        self.linear_velocity: float = 0
        self.angular_velocity: float = 0

    async def configure(self):
        await super().configure()
        raise Exception('there is no communication for configuring hardware')

    async def restart(self):
        await super().restart()
        raise Exception('there is no communication for restarting hardware')

    async def drive(self, linear: float, angular: float):
        await super().drive(linear, angular)
        self.linear_velocity = linear
        self.angular_velocity = angular

    async def stop(self):
        await super().stop()
        self.linear_velocity = 0
        self.angular_velocity = 0

    async def update(self):
        await super().update()
        self.world.robot.odometry.append(Velocity(
            linear=self.linear_velocity,
            angular=self.angular_velocity,
            time=self.world.time,
        ))
