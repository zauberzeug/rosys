from typing import Optional

from ..communication import Communication
from ..helpers import sleep
from ..world import Velocity, World
from . import CommunicatingHardware


class RobotBrain(CommunicatingHardware):

    def __init__(self, world: World, communication: Communication):
        super().__init__(world, communication)
        self.waiting_list: dict[str, Optional[str]] = {}

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
            line = self.check(await self.communication.read())
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
                if self.world.robot.clock_offset is None:
                    continue
                self.world.robot.hardware_time = millis / 1000 + self.world.robot.clock_offset
                self.parse_core(words)
            self.parse_line(line)
        if millis is not None:
            self.world.robot.clock_offset = self.world.time - millis / 1000

    def parse_core(self, words: list[str]):
        self.world.robot.odometry.append(Velocity(
            linear=float(words.pop(0)),
            angular=float(words.pop(0)),
            time=self.world.robot.hardware_time,
        ))

    def parse_line(self, line: str):
        pass

    async def send(self, msg: str):
        await self.communication.send_async(self.augment(msg))

    async def send_and_await(self, msg: str, ack: str, *, timeout: float = float('inf')) -> Optional[str]:
        self.waiting_list[ack] = None
        await self.send(msg)
        t0 = self.world.time
        while self.waiting_list.get(ack) is None and self.world.time < t0 + timeout:
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
