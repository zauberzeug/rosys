import asyncio
from typing import Optional
from .. import Runtime, run
from ..world import Robot, World


class TestRuntime(Runtime):
    __test__ = False

    def __init__(self, world: Optional[World] = None):
        if world is None:
            world = World(robot=Robot())
        super().__init__(world)
        self.world.set_time(0)  # NOTE in tests we start at zero for better reading

        from .helper import set_global_runtime  # NOTE import here to avoid PytestAssertRewriteWarning
        set_global_runtime(self)
        self.exception = None

    async def forward(self, seconds: Optional[float] = None, until: Optional[float] = None, dt: float = 0.01):
        # NOTE we start runtime here because this makes it easy in the tests to prepare it beforehand
        if not self.tasks:
            await self.startup()

        assert seconds or until
        end_time = self.world.time + seconds if seconds is not None else until
        self.log.info(f'-------------------> forwarding to {round(end_time,2)}')
        while self.world.time <= end_time:
            if not run.running_processes:
                self.world.set_time(self.world.time + dt)
                await asyncio.sleep(0)
            else:
                await asyncio.sleep(0.01)
            if self.exception is not None:
                raise RuntimeError(f'error while forwarding time {dt} s') from self.exception
        self.log.info(f'-------------------> now it\'s {round(self.world.time,2)}')

    def handle_exception(self, ex: Exception):
        super().handle_exception(ex)
        self.exception = ex
