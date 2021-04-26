import sys
import time
import asyncio
import task_logger
from typing import get_type_hints
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

    def automate(self, async_func):

        params = []
        for name, type_ in get_type_hints(async_func).items():
            for obj in [self.world] + self.actors:
                if isinstance(obj, type_):
                    params.append(obj)
                    break
            else:
                raise Exception(f'parameter "{name}" of type {type_} is unknown')

        task_logger.create_task(async_func(*params))
