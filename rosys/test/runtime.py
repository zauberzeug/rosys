import asyncio
from typing import Callable, Optional, Union
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

    async def forward(self,
                      seconds: Optional[float] = None,
                      *,
                      until: Optional[Union[int, float, Callable]] = None,
                      x: Optional[float] = None,
                      y: Optional[float] = None,
                      tolerance: float = 0.1,
                      dt: float = 0.01,
                      timeout: float = 100):
        # NOTE we start runtime here because this makes it easy in the tests to prepare it beforehand
        if not self.tasks:
            await self.startup()

        start_time = self.world.time
        pose = self.world.robot.prediction
        if seconds is not None:
            msg = f'forwarding {seconds=}'
            def condition(): return self.world.time >= start_time + seconds
        elif isinstance(until, int) or isinstance(until, float):
            msg = f'forwarding {until=}'
            def condition(): return self.world.time >= until
        elif isinstance(until, Callable):
            msg = f'forwarding {until=}'
            condition = until
        elif x is not None and y is not None:
            msg = f'forwarding to {x=} and {y=}'
            def condition(): return (pose.x - x)**2 + (pose.y - y)**2 < tolerance**2
        elif x is not None:
            msg = f'forwarding to {x=}'
            def condition(): return abs(pose.x - x) < tolerance
        elif y is not None:
            msg = f'forwarding to {y=}'
            def condition(): return abs(pose.y - y) < tolerance
        else:
            raise Exception('invalid arguments')

        self.log.info(f'\033[94m{msg}\033[0m')
        while not condition():
            if not run.running_processes:
                self.world.set_time(self.world.time + dt)
                await asyncio.sleep(0)
            else:
                await asyncio.sleep(0.01)
            if self.exception is not None:
                raise RuntimeError(f'error while forwarding time {dt} s') from self.exception
            if self.world.time > start_time + timeout:
                raise TimeoutError(f'condition not met in time')

    def handle_exception(self, ex: Exception):
        super().handle_exception(ex)
        self.exception = ex
