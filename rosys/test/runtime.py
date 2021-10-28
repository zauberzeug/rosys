import asyncio
from typing import Callable
from rosys.runtime import Runtime
from rosys.world.mode import Mode
from rosys.world.robot import Robot
from rosys.world.world import World, WorldState


class TestRuntime(Runtime):
    is_time_running: bool = True

    def __init__(self, world: World = None):
        if world is None:
            world = World(mode=Mode.TEST, state=WorldState.RUNNING, robot=Robot())
        super().__init__(world)
        self.world.set_time(0)  # NOTE in tests we start at zero for better reading

        from .helper import set_global_runtime  # NOTE import here to avoid PytestAssertRewriteWarning
        set_global_runtime(self)

    async def forward(self, seconds, dt=0.01):
        # NOTE we start runtime here because this makes it easy in the tests to prepare it beforehand
        if not self.tasks:
            for actor in self.actors:
                actor.sleep = self.sleep
                actor.run_in_executor = self.run_in_executor
            await self.start()

        end_time = self.world.time + seconds
        self.log.info(f'-------------------> forwarding to {round(end_time,2)}')
        while self.world.time <= end_time:
            if self.is_time_running:
                self.world.set_time(self.world.time + dt)
                await asyncio.sleep(0)
            else:
                await asyncio.sleep(0.01)
        self.log.info(f'-------------------> now it\'s {round(self.world.time,2)}')

    async def sleep(self, seconds: float):
        sleep_end_time = self.world.time + seconds
        while self.world.time <= sleep_end_time:
            await asyncio.sleep(0)

    async def run_in_executor(self, callback: Callable):
        try:
            self.is_time_running = False
            loop = asyncio.get_running_loop()
            return await loop.run_in_executor(None, callback)
        finally:
            self.is_time_running = True
