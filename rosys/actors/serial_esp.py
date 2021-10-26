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
        aioserial.AioSerial('/dev/esp', baudrate=115200)  # NOTE try to open serial port (factory needs this)
        self.aioserial = None
        self.remainder = ''

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
        try:
            # HACK writing synchronous because the async way contains a lock which does not work with our pause/resume of automations
            self.aioserial.write(line.encode())
        finally:
            await self.sleep(0)  # make sure we let other coroutines do their work on the event loop

    def is_open(self):
        if self.aioserial is None:
            self.aioserial = aioserial.AioSerial('/dev/esp', baudrate=115200)
        return self.aioserial.isOpen()
