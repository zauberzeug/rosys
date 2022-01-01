import asyncio
from typing import Callable, Optional
from .. import Runtime
from ..world import AutomationState, Mode, Robot, World


class TestRuntime(Runtime):
    __test__ = False
    is_time_running: bool = True

    def __init__(self, world: Optional[World] = None):
        if world is None:
            world = World(mode=Mode.TEST, automation_state=AutomationState.RUNNING, robot=Robot())
        super().__init__(world)
        self.world.set_time(0)  # NOTE in tests we start at zero for better reading

        from .helper import set_global_runtime  # NOTE import here to avoid PytestAssertRewriteWarning
        set_global_runtime(self)
        self.exception = None

    async def forward(self, seconds, dt=0.01):
        # NOTE we start runtime here because this makes it easy in the tests to prepare it beforehand
        if not self.tasks:
            for actor in self.actors:
                actor.sleep = self.sleep
                actor.run_cpu_bound = self.run_cpu_bound
            await self.startup()

        end_time = self.world.time + seconds
        self.log.info(f'-------------------> forwarding to {round(end_time,2)}')
        while self.world.time <= end_time:
            if self.is_time_running:
                self.world.set_time(self.world.time + dt)
                await asyncio.sleep(0)
            else:
                await asyncio.sleep(0.01)
            if self.exception is not None:
                raise RuntimeError(f'error while forwarding time {dt} s') from self.exception

        self.log.info(f'-------------------> now it\'s {round(self.world.time,2)}')

    async def sleep(self, seconds: float):
        sleep_end_time = self.world.time + seconds
        while self.world.time <= sleep_end_time:
            await asyncio.sleep(0)

    async def run_cpu_bound(self, callback: Callable, *args: any):
        try:
            self.is_time_running = False
            loop = asyncio.get_running_loop()
            return await loop.run_in_executor(None, callback, *args)
        finally:
            self.is_time_running = True

    def handle_exception(self, ex: Exception):
        super().handle_exception(ex)
        self.exception = ex
