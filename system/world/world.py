import asyncio
import task_logger
from typing import Coroutine
from pydantic import BaseModel
from world.robot import Robot
from world.clock import Clock
import task_logger


class World(BaseModel):

    clock: Clock
    robot: Robot

    async def run(self):
        while True:
            self.loop()
            await asyncio.sleep(self.clock.interval)

    async def simulate(self, seconds):

        end = self.clock.time + seconds
        while self.clock.time < end:
            self.loop()
            await asyncio.sleep(0)  # give other coroutines a chance to execute their code

    def loop(self):

        self.clock.loop()
        self.robot.loop(self.clock.interval)

    def automate(_, coro: Coroutine):
        print('running automation coroutine', flush=True)
        task_logger.create_task(coro)
