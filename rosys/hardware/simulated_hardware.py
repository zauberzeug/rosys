from ..actors.odometer import Odometer
from ..core import core
from .hardware import Hardware


class SimulatedHardware(Hardware):

    def __init__(self, odometer: Odometer):
        super().__init__(odometer)
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
        self.odometer.add_odometry(self.linear_velocity, self.angular_velocity, core.time)
