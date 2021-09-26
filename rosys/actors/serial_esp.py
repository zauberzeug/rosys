import aioserial
from operator import ixor
from functools import reduce
from .esp import Esp
from ..world.world import World


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

        millis = None
        while '\n' in self.remainder:
            line, self.remainder = self.remainder.split('\n', 1)

            if line.startswith("\x1b[0;32m"):
                self.log.warning(line)
                continue  # NOTE: ignore green log messages

            if '^' in line:
                line, checksum = line.split('^')
                if reduce(ixor, map(ord, line)) != int(checksum):
                    self.log.warning('Checksum failed')
                    continue

            if not line.startswith("esp "):
                self.log.warning(line)
                continue  # NOTE: ignore all messages but esp status

            try:
                words = line.split()[1:]
                millis = float(words.pop(0))
                if world.robot.clock_offset is None:
                    continue
                world.robot.hardware_time = millis / 1000 + world.robot.clock_offset
                for group in world.robot.hardware:
                    if group.output:
                        group.parse(words, world)
            except (IndexError, ValueError):
                self.log.warning(f'Error parsing serial message "{line}"')

        if millis is not None:
            world.robot.clock_offset = world.time - millis / 1000

    async def send_async(self, line):
        if not self.aioserial.isOpen():
            return

        line = f'{line}^{reduce(ixor, map(ord, line))}\n'
        await self.aioserial.write_async(line.encode())
