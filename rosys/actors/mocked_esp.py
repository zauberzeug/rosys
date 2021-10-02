import asyncio
import numpy as np
from ..world.velocity import Velocity
from ..world.world import World
from .esp import Esp


class MockedEsp(Esp):
    interval: float = 0.01

    def __init__(self, world: World):
        super().__init__()

        x = [point[0] for point in world.robot.shape.outline]
        self.width = max(x) - min(x)
        self.linear_velocity: float = 0
        self.angular_velocity: float = 0

    async def step(self, world: World):
        velocity = Velocity(linear=self.linear_velocity, angular=self.angular_velocity, time=world.time)
        world.robot.odometry.append(velocity)
        world.robot.battery = 25.0 + np.sin(0.1 * world.time) + 0.02 * np.random.randn()
        world.robot.temperature = np.random.uniform(34, 35)

    async def send_async(self, line):
        if line.startswith("wheels power "):
            left = float(line.split()[2].split(',')[0])
            right = float(line.split()[2].split(',')[1])
            self.linear_velocity = (left + right) / 2.0
            self.angular_velocity = (right - left) / self.width / 2.0

        if line.startswith("wheels speed "):
            self.linear_velocity = float(line.split()[2].split(',')[0])
            self.angular_velocity = float(line.split()[2].split(',')[1])

        await asyncio.sleep(0)
