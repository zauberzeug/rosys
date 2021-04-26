import sys
import time
import asyncio
import task_logger
from typing import Coroutine
from actors.actor import Actor
from actors.esp import SerialEsp, MockedEsp
from actors.odometer import Odometer
from world.world import World
from world.robot import Robot
from world.mode import Mode
from typing import get_type_hints


class Runtime:

    def __init__(self, mode: Mode):

        self.world = World(
            mode=mode,
            time=time.time(),
            robot=Robot(),
        )

        self.esp = SerialEsp(self.world) if mode == Mode.REAL else MockedEsp(self.world)
        self.odometer = Odometer(self.world)

        self.actors = [
            self.esp,
            self.odometer,
        ]

    async def run(self, seconds: float = sys.maxsize):

        end_time = self.world.time + seconds
        tasks = [task_logger.create_task(self.update_time(end_time))]

        for actor in self.actors:
            if hasattr(actor, "every_10_ms"):
                tasks.append(task_logger.create_task(self.automate(actor.every_10_ms, 0.01, end_time)))

        await asyncio.gather(*tasks)

    async def automate(self, step, interval, end_time):
        while self.world.time < end_time:
            await step()
            await self.sleep(interval)

    async def update_time(self, end_time):
        while True:
            self.world.time = self.world.time + 0.01 if self.world.mode == Mode.TEST else time.time()
            if self.world.time > end_time:
                break
            await asyncio.sleep(0 if self.world.mode == Mode.TEST else 0.01)

    async def sleep(self, seconds: float):
        if self.world.mode == Mode.TEST:
            end_time = self.world.time + seconds
            while self.world.time < end_time:
                await asyncio.sleep(0)
        else:
            await asyncio.sleep(seconds)

    def automate_old(self, async_func):

        params = []
        for name, type_ in get_type_hints(async_func).items():
            p = None
            if type_ is World:
                p = self.world
            for actor in self.actors:
                if isinstance(actor, type_):
                    p = actor
            if p is not None:
                params.append(p)
            else:
                raise Exception(f'parameter "{name}" of type {type_} is unknown')

        task_logger.create_task(async_func(*params))
