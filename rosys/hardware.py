from typing import Optional
import numpy as np
from .communication.communication import Communication
from .communication.communication_factory import CommunicationFactory
from .world.mode import Mode
from .world.velocity import Velocity
from .world.world import World


class Hardware:

    def __init__(self, world: World, communication: Optional[Communication] = None):
        self.world = world
        if communication is None and world.mode == Mode.REAL:
            self.communication = CommunicationFactory.create()
            x = [point[0] for point in self.world.robot.shape.outline]
            self.width = max(x) - min(x)
            self.linear_velocity: float = 0
            self.angular_velocity: float = 0
        else:
            self.communication = communication

    async def configure(self, filepath: str):
        if self.communication:
            with open(filepath) as f:
                await self.communication.send_async(f'!-')
                for line in f.readlines():
                    await self.communication.send_async(f'!+{line}')
                await self.communication.send_async(f'!.')
                await self.restart()
        else:
            raise Exception('there is no communication for configuring hardware')

    async def restart(self):
        if self.communication:
            await self.communication.send_async(f'core.restart()')
        else:
            raise Exception('there is no communication for restarting hardware')

    async def drive(self, linear: float, angular: float):
        if self.communication:
            await self.communication.send_async(f'wheels.speed({linear}, {angular})')
        else:
            self.linear_velocity = linear
            self.angular_velocity = angular

    async def stop(self):
        if self.communication:
            await self.communication.send_async(f'wheels.stop()')
        else:
            self.linear_velocity = 0
            self.angular_velocity = 0

    async def update(self):
        if self.communication:
            line = await self.communication.read()
            if line is not None:
                self.parse(line.split())
        else:
            velocity = Velocity(linear=self.linear_velocity, angular=self.angular_velocity, time=self.world.time)
            self.world.robot.odometry.append(velocity)
            self.world.robot.battery = 25.0 + np.sin(0.1 * self.world.time) + 0.02 * np.random.randn()
            self.world.robot.temperature = np.random.uniform(34, 35)

    def parse(self, words: list[str]):
        self.linear_velocity = float(words.pop(0))
        self.angular_velocity = float(words.pop(1))
