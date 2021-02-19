from pydantic import BaseModel
from .clock import Clock
import asyncio


class World(BaseModel):

    clock: Clock

    async def run(self):

        while True:
            self.loop()
            await asyncio.sleep(self.clock.interval)

    async def simulate(self, seconds):

        end = self.clock.time + seconds
        while self.clock.time < end:
            self.loop()

    def loop(self):

        self.clock.loop()
