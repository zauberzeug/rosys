import time
import asyncio
from actors.actor import Actor
from world.mode import Mode


class Clock(Actor):

    async def step(self):

        self.world.time = time.time()
        await asyncio.sleep(0.01 if self.world.mode == Mode.REAL else 0.1)


class TestClock(Actor):

    async def step(self):

        self.world.time += 0.01
        await asyncio.sleep(0)
