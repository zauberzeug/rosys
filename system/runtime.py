import sys
import time
import asyncio
from actors.esp import SerialEsp, MockedEsp
from actors.clock import Clock, TestClock
from actors.odometer import Odometer
from world.world import World
from world.robot import Robot
from world.mode import Mode


class Runtime:

    def __init__(self, mode: Mode):

        self.world = World(
            mode=mode,
            time=time.time(),
            robot=Robot(),
        )

        self.esp = SerialEsp(self.world) if mode == Mode.REAL else MockedEsp(self.world)
        self.clock = TestClock(self.world) if mode == Mode.TEST else Clock(self.world)
        self.odometer = Odometer(self.world)

        self.actors = [
            self.esp,
            self.odometer,
            self.clock,
        ]

    async def run(self, seconds: float = sys.maxsize):

        end_time = self.world.time + seconds
        await asyncio.gather(*[actor.run(end_time) for actor in self.actors])
