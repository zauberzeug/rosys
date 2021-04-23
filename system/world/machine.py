from typing import Any
from pydantic.main import BaseModel, PrivateAttr
import aioserial
import asyncio
import time
from world.velocity import Velocity


class Machine(BaseModel):

    time: float = 0


class SerialMachine(Machine):

    port: str
    _aioserial_instance: Any = PrivateAttr()
    _time_offset: float = PrivateAttr(0)

    def __init__(self):

        super().__init__()

        self._aioserial_instance = aioserial.AioSerial(self.port, baudrate=115200)

    async def read(self) -> Velocity:

        try:
            line = (await self._aioserial_instance.readline_async()).decode().strip()
        except:
            raise IOError('Error reading from serial')

        if '^' in line:
            line, check = line.split('^')
            checksum = 0
            for c in line:
                checksum ^= ord(c)
            if checksum != int(check):
                return

        try:
            words = line.split()[1:]
            millis = float(words.pop(0))
            linear = float(words.pop(0))
            angular = float(words.pop(0))
        except (IndexError, ValueError):
            raise IOError(f'Error parsing serial message "{line}"')

        if self._time_offset is None:
            self._time_offset = time.time()
        self.time = self._time_offset + millis / 1000.0

        return Velocity(linear=linear, angular=angular)

    async def send(self, line):

        checksum = 0
        for c in line:
            checksum ^= ord(c)
        line += '^%d\n' % checksum
        await self._aioserial_instance.write_async(line.encode())


class MockedMachine(Machine):

    width: float = 0
    realtime: bool = False
    _velocity: Velocity = PrivateAttr(Velocity(linear=0, angular=0))
    _delay: float = PrivateAttr()

    def __init__(self, width: float, delay: float = 0, realtime: bool = False):

        super().__init__()
        self.width = width
        self._delay = delay

    async def read(self) -> Velocity:

        if self.realtime:
            self.time = time.time()
        else:
            self.time += 0.1
        await asyncio.sleep(self._delay)

        return self._velocity

    async def send(self, line):

        if line.startswith("drive pw "):
            left = float(line.split()[2].split(',')[0])
            right = float(line.split()[2].split(',')[1])
            self._velocity.linear = (left + right) / 2.0
            self._velocity.angular = (right - left) / self.width / 2.0

        if line.startswith("drive speed "):
            self._velocity.linear = float(line.split()[2].split(',')[0])
            self._velocity.angular = float(line.split()[2].split(',')[1])
