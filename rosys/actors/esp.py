import asyncio

from justpy.justpy import create_component_file_list
from rosys import task_logger
import time
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
