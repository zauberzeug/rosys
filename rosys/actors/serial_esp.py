import aioserial
from functools import reduce
from .esp import Esp
from ..world.world import World
from operator import ixor


class SerialEsp(Esp):
    interval: float = 0.01

    def __init__(self):
        super().__init__()

        self.aioserial = aioserial.AioSerial('/dev/esp', baudrate=115200)
        self.remainder = ''

    def pause(self):
        self.aioserial.close()

    def resume(self):
        self.aioserial.open()

    async def step(self, world: World):
        if not self.aioserial.isOpen():
            return

        try:
            self.remainder += self.aioserial.read_all().decode()
        except:
            self.log.warning('Error reading from serial')
            return

        self.remainder = self.parse(self.remainder, world)

    async def send_async(self, line):
        if not self.aioserial.isOpen():
            return

        line = f'{line}^{reduce(ixor, map(ord, line))}\n'
        await self.aioserial.write_async(line.encode())
