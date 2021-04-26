from actors.guard import Guard
from actors.actor import Actor
import sys
import time
import asyncio
import task_logger
from typing import get_type_hints
from actors.esp import SerialEsp, MockedEsp
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
        self.actors = []

        self.esp = self.add(SerialEsp if mode == Mode.REAL else MockedEsp)
        self.odometer = self.add(Odometer)
        self.guard = self.add(Guard)

    def add(self, actor_type):

        params = self.get_params(actor_type.__init__)
        actor = actor_type(*params)
        self.actors.append(actor)

        if hasattr(actor, "once"):
            task_logger.create_task(actor.once(*self.get_params(actor.once)))

        return actor

    async def run(self, seconds: float = sys.maxsize):

        end_time = self.world.time + seconds
        tasks = [task_logger.create_task(self.advance_time(end_time))]

        for actor in self.actors:
            if hasattr(actor, "every_10_ms"):
                tasks.append(task_logger.create_task(self.repeat(actor.every_10_ms, 0.01, end_time)))
            if hasattr(actor, "every_100_ms"):
                tasks.append(task_logger.create_task(self.repeat(actor.every_100_ms, 0.1, end_time)))

        await asyncio.gather(*tasks)

    async def repeat(self, step, interval, end_time):

        params = self.get_params(step)
        while self.world.time < end_time:
            await step(*params)
            await self.guard.sleep(interval)

    async def advance_time(self, end_time):

        while True:
            self.world.time = self.world.time + 0.01 if self.world.mode == Mode.TEST else time.time()
            if self.world.time > end_time:
                break
            await asyncio.sleep(0 if self.world.mode == Mode.TEST else 0.01)

    def get_params(self, func):

        params = []
        for name, type_ in get_type_hints(func).items():
            for obj in [self.world] + self.actors:
                if isinstance(obj, type_):
                    params.append(obj)
                    break
            else:
                raise Exception(f'parameter "{name}" of type {type_} is unknown')

        return params
