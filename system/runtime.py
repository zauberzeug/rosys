import sys
import time
import asyncio
from enum import Enum
from actors.serial import Serial, MockedSerial
from actors.clock import Clock, TestClock
from actors.odometer import Odometer
from world.world import World
from world.robot import Robot


class Mode(Enum):
    TEST = 1
    SIMULATION = 2
    REAL = 3


class Runtime:

    def __init__(self, mode: Mode):

        self.world = World(
            time=time.time(),
            robot=Robot(),
        )

        self.serial = Serial(self.world) if mode == Mode.REAL else MockedSerial(self.world)
        self.clock = TestClock(self.world) if mode == Mode.TEST else Clock(self.world)
        self.odometer = Odometer(self.world)

        self.actors = [
            self.clock,
            self.serial,
            self.odometer,
        ]

    async def run(self, seconds: float = sys.maxsize):

        await asyncio.gather(*[actor.run(seconds) for actor in self.actors])
