from actors.odometer import Odometer
import aioserial
import time
import numpy as np
from world.world import World
from world.velocity import Velocity
from actors.actor import Actor


class Esp(Actor):

    async def drive(self, linear: float, angular: float):

        await self.send('drive speed %.3f,%.3f' % (linear, angular))

    async def power(self, left: float, right: float):

        await self.send('drive pw %.3f,%.3f' % (left, right))


class SerialEsp(Esp):

    def __init__(self):
        
        self.aioserial = aioserial.AioSerial('/dev/esp', baudrate=115200)

    async def every_10_ms(self, world: World, odometer: Odometer):

        try:
            line = (await self.aioserial.readline_async()).decode().strip()
        except:
            raise IOError('Error reading from serial')

        if line.startswith("\x1b[0;32m"):
            return  # NOTE: ignore green log messages

        if '^' in line:
            line, check = line.split('^')
            checksum = 0
            for c in line:
                checksum ^= ord(c)
            if checksum != int(check):
                return

        try:
            words = line.split()[2:]
            # TODO: read millis and compare with world time
            linear = float(words.pop(0))
            angular = float(words.pop(0))
            temperature = float(words.pop(0))
            battery = float(words.pop(0))
        except (IndexError, ValueError):
            raise IOError(f'Error parsing serial message "{line}"')

        world.robot.velocity.linear = linear
        world.robot.velocity.angular = angular
        odometer.update(world.robot.velocity, world)

        world.robot.battery = battery
        world.robot.temperature = temperature

    async def send(self, line):

        checksum = 0
        for c in line:
            checksum ^= ord(c)
        line += '^%d\n' % checksum
        await self.aioserial.write_async(line.encode())


class MockedEsp(Esp):

    width: float = 0.5
    _velocity: Velocity = Velocity(linear=0, angular=0)

    async def every_10_ms(self, world: World, odometer: Odometer):

        world.robot.velocity.linear = self._velocity.linear
        world.robot.velocity.angular = self._velocity.angular
        odometer.update(self._velocity, world)
        world.robot.battery = 25.0 + np.sin(0.1 * time.time()) + 0.02 * np.random.randn()
        world.robot.temperature = np.random.uniform(34, 35)

    async def send(self, line):

        if line.startswith("drive pw "):
            left = float(line.split()[2].split(',')[0])
            right = float(line.split()[2].split(',')[1])
            self._velocity.linear = (left + right) / 2.0
            self._velocity.angular = (right - left) / self.width / 2.0

        if line.startswith("drive speed "):
            self._velocity.linear = float(line.split()[2].split(',')[0])
            self._velocity.angular = float(line.split()[2].split(',')[1])
