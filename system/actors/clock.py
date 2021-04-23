import time
import asyncio
from actors.actor import Actor


class Clock(Actor):

    async def step(self):

        self.world.time = time.time()
        await asyncio.sleep(0.01)
