from typing import Optional

from .. import event
from ..actors.odometer import Odometer
from ..communication import Communication
from ..core import core
from ..helpers import sleep
from ..lifecycle import on_shutdown, on_startup
from . import AppControls, CommunicatingHardware


class RobotBrain(CommunicatingHardware):

    def __init__(self, odometer: Odometer, communication: Communication):
        super().__init__(odometer, communication)
        self.app_controls = AppControls(self)
        event.register(event.Id.NEW_NOTIFICATION, self.app_controls.notify)
        self.waiting_list: dict[str, Optional[str]] = {}

        self.clock_offset: Optional[float] = None
        self.hardware_time: Optional[float] = None

        on_startup(self.app_controls.sync)
        on_shutdown(self.app_controls.clear)
        on_shutdown(lambda: self.drive(0, 0))

    async def configure(self, filepath: str = 'lizard.txt'):
        await super().configure()
        await self.send(f'!-')
        with open(filepath) as f:
            for line in f.read().splitlines():
                await self.send(f'!+{line}')
        await self.send(f'!.')
        await self.restart()

    async def restart(self):
        await super().restart()
        await self.send(f'core.restart()')

    async def drive(self, linear: float, angular: float):
        await super().drive(linear, angular)
        await self.send(f'wheels.speed({linear}, {angular})')

    async def stop(self):
        await super().stop()
        await self.send('wheels.off()')

    async def update(self):
        await super().update()
        millis = None
        while True:
            unchecked = await self.communication.read()
            line = self.check(unchecked)
            if line is None or not line:
                break
            words = line.split()
            if not words:
                continue
            first = words.pop(0)
            if first in self.waiting_list:
                self.waiting_list[first] = line
                continue
            if first == 'core':
                millis = float(words.pop(0))
                if self.clock_offset is None:
                    continue
                self.hardware_time = millis / 1000 + self.clock_offset
                self.parse_core(words)
            self.parse_line(line)
        if millis is not None:
            self.clock_offset = core.time - millis / 1000

    def parse_core(self, words: list[str]) -> None:
        self.odometer.add_odometry(float(words.pop(0)), float(words.pop(0)), self.hardware_time)

    def parse_line(self, line: str) -> None:
        if self.app_controls is not None:
            self.app_controls.parse(line)

    async def send(self, msg: str):
        await self.communication.send_async(self.augment(msg))

    async def send_and_await(self, msg: str, ack: str, *, timeout: float = float('inf')) -> Optional[str]:
        self.waiting_list[ack] = None
        await self.send(msg)
        t0 = core.time
        while self.waiting_list.get(ack) is None and core.time < t0 + timeout:
            await sleep(0.1)
        return self.waiting_list.pop(ack) if ack in self.waiting_list else None

    @staticmethod
    def augment(line: str) -> str:
        checksum = 0
        for c in line:
            checksum ^= ord(c)
        return f'{line}@{checksum:02x}'

    @staticmethod
    def check(line: Optional[str]) -> str:
        if line is None:
            return ""
        if line[-3:-2] == '@':
            check = int(line[-2:], 16)
            line = line[:-3]
            checksum = 0
            for c in line:
                checksum ^= ord(c)
            if checksum != check:
                return None
        return line
