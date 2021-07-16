import asyncio
import logging
import aioserial
from operator import ixor
from functools import reduce
import numpy as np
from ..world.world import World
from ..world.velocity import Velocity
from .actor import Actor


class Esp(Actor):

    async def drive(self, linear: float, angular: float):

        await self.send('drive speed %.3f,%.3f' % (linear, angular))

    async def power(self, left: float, right: float):

        await self.send('drive pw %.3f,%.3f' % (left, right))


class SerialEsp(Esp):

    interval: float = 0.01

    def __init__(self):

        self.aioserial = aioserial.AioSerial('/dev/esp', baudrate=115200)
        self.remainder = ''

    async def step(self, world: World):

        try:
            self.remainder += self.aioserial.read_all().decode()
        except:
            logging.warning('Error reading from serial')
            return

        millis = None
        while '\n' in self.remainder:

            line, self.remainder = self.remainder.split('\n', 1)

            if line.startswith("\x1b[0;32m"):
                logging.warning(line)
                continue  # NOTE: ignore green log messages

            if '^' in line:
                line, checksum = line.split('^')
                if reduce(ixor, map(ord, line)) != int(checksum):
                    logging.warning('Checksum failed')
                    continue

            if not line.startswith("esp "):
                logging.warning(line)
                continue  # NOTE: ignore all messages but esp status

            try:
                words = line.split()[1:]
                millis = float(words.pop(0))
                linear = float(words.pop(0))
                angular = float(words.pop(0))
                world.robot.temperature = float(words.pop(0))
                world.robot.battery = float(words.pop(0))
                if world.robot.clock_offset is not None:
                    time = millis / 1000 + world.robot.clock_offset
                    world.robot.odometry.append(Velocity(linear=linear, angular=angular, time=time))
            except (IndexError, ValueError):
                logging.warning(f'Error parsing serial message "{line}"')

        if millis is not None:
            world.robot.clock_offset = world.time - millis / 1000

    async def send(self, line):

        line = f'{line}^{reduce(ixor, map(ord, line))}\n'
        self.aioserial.write(line.encode())
        await asyncio.sleep(0)


class MockedEsp(Esp):

    interval: float = 0.01

    def __init__(self, world: World):

        x = [point[0] for point in world.robot.shape.outline]
        self.width = max(x) - min(x)
        self.linear_velocity: float = 0
        self.angular_velocity: float = 0

    async def step(self, world: World):

        velocity = Velocity(linear=self.linear_velocity, angular=self.angular_velocity, time=world.time)
        world.robot.odometry.append(velocity)
        world.robot.battery = 25.0 + np.sin(0.1 * world.time) + 0.02 * np.random.randn()
        world.robot.temperature = np.random.uniform(34, 35)

    async def send(self, line):

        if line.startswith("drive pw "):
            left = float(line.split()[2].split(',')[0])
            right = float(line.split()[2].split(',')[1])
            self.linear_velocity = (left + right) / 2.0
            self.angular_velocity = (right - left) / self.width / 2.0

        if line.startswith("drive speed "):
            self.linear_velocity = float(line.split()[2].split(',')[0])
            self.angular_velocity = float(line.split()[2].split(',')[1])

        await asyncio.sleep(0)
