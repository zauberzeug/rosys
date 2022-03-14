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
        robot = self.world.robot
        if seconds is not None:
            msg = f'forwarding {seconds=}'
            def condition(): return self.world.time >= start_time + seconds
            timeout = max(timeout, seconds)
        elif isinstance(until, int) or isinstance(until, float):
            msg = f'forwarding {until=}'
            def condition(): return self.world.time >= until
            timeout = max(timeout, until - start_time)
        elif isinstance(until, Callable):
            msg = f'forwarding {until=}'
            condition = until
        elif x is not None and y is not None:
            msg = f'forwarding to {x=} and {y=}'
            def condition(): return (robot.prediction.x - x)**2 + (robot.prediction.y - y)**2 < tolerance**2
        elif x is not None:
            msg = f'forwarding to {x=}'
            def condition(): return abs(robot.prediction.x - x) < tolerance
        elif y is not None:
            msg = f'forwarding to {y=}'
            def condition(): return abs(robot.prediction.y - y) < tolerance
        else:
            raise Exception('invalid arguments')

        self.log.info(f'\033[94m{msg}\033[0m')
        while not condition():
            if self.world.time > start_time + timeout:
                raise TimeoutError(f'condition took more than {timeout} s')
            if not run.running_processes:
                self.world.set_time(self.world.time + dt)
                await asyncio.sleep(0)
            else:
                await asyncio.sleep(0.01)
            if self.exception is not None:
                raise RuntimeError(f'error while forwarding time {dt} s') from self.exception

    def handle_exception(self, ex: Exception):
        super().handle_exception(ex)
        self.exception = ex
