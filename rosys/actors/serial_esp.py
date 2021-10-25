import aioserial
from functools import reduce
from operator import ixor
from ..world.world import World
from .esp import Esp
import asyncio


class SerialEsp(Esp):
    interval: float = 0.01

    def __init__(self):
        super().__init__()

        self.aioserial = None
        self.remainder = ''

    def pause(self):
        if not self.is_open():
            return
        self.aioserial.close()

    def resume(self):
        if not self.is_open():
            return
        self.aioserial.open()

    async def step(self, world: World):
        if not self.is_open():
            return

        try:
            self.remainder += self.aioserial.read_all().decode()
        except:
            self.log.warning('Error reading from serial')
            return

        self.remainder = self.parse(self.remainder, world)

    async def send_async(self, line):
        if not self.is_open():
            return

        line = f'{line}^{reduce(ixor, map(ord, line))}\n'
        await self.aioserial.write_async(line.encode())

    def is_open(self):
        if self.aioserial is None:
            self.aioserial = aioserial.AioSerial('/dev/esp', baudrate=115200)
        return self.aioserial.isOpen()
