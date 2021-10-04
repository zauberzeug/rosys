import time
from operator import ixor
from functools import reduce
from .. import task_logger
from ..world.world import World
from ..world.hardware import HardwareGroup
from .actor import Actor


class Esp(Actor):

    async def drive(self, linear: float, angular: float):
        await self.send_async('wheels speed %.3f,%.3f' % (linear, angular))

    async def power(self, left: float, right: float):
        await self.send_async('wheels power %.3f,%.3f' % (left, right))

    def configure(self, hardware: list[HardwareGroup]):
        task_logger.create_task(self.configure_async(hardware))

    async def configure_async(self, hardware: list[HardwareGroup]):
        await self.send_async('esp erase')

        for group in hardware:
            for command in group.commands:
                await self.send_async('+' + command)
                time.sleep(0.1)

        output_modules = ','.join(group.name for group in hardware if group.output)
        await self.send_async(f'+set esp.outputModules={output_modules}')
        await self.send_async('+esp unmute')
        await self.send_async('+set esp.ready=1')
        await self.send_async('+set esp.24v=1')
        await self.send_async('esp restart')

    def parse(self, messages: str, world: World):
        '''Parses the messages received from esp messages and writes the data into the world.
        Note: an incomplete message at the end is not parsed but returend to be completed later.'''

        millis = None
        while '\n' in messages:
            line, messages = messages.split('\n', 1)
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
                self.log.warning(f'Error parsing esp message "{line}"')

        if millis is not None:
            world.robot.clock_offset = world.time - millis / 1000

        return messages
