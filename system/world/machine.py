from typing import Any
from pydantic.main import BaseModel, PrivateAttr
import aioserial
import time
from world.velocity import Velocity


class Machine(BaseModel):

    port: str
    time: float = 0
    _aioserial_instance: Any = PrivateAttr()
    _time_offset: float = PrivateAttr(0)

    def __init__(self, *args, **kwargs):

        super().__init__(*args, **kwargs)

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

    def send(self, line):

        checksum = 0
        for c in line:
            checksum ^= ord(c)
        line += '^%d\n' % checksum
        self._aioserial_instance.write(line.encode())
