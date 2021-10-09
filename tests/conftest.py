from typing import Generator
import pytest
from rosys.runtime import Runtime, Mode
from rosys.world.world import World, WorldState
from rosys.world.robot import Robot
from rosys.world.marker import Marker
import asyncio

import icecream
icecream.install()

pytest.register_assert_rewrite("tests.helper")


@pytest.fixture
async def runtime() -> Generator:
    runtime = Runtime(World(
        mode=Mode.TEST,
        state=WorldState.RUNNING,
        robot=Robot(marker=Marker(points={'front': (0.12, 0), 'back': (-0.12, 0)}, height=0.58)),
    )).with_cameras()

    from tests.helper import set_global_runtime
    set_global_runtime(runtime)

    runtime.world.set_time(0)  # NOTE in tests we start at zero for better reading

    async def forward(seconds, dt=0.01):
        # NOTE we start runtime here because this makes it easy in the tests to prepare it beforehand
        if not runtime.tasks:
            await runtime.run()

        end_time = runtime.world.time + seconds
        while runtime.world.time <= end_time:
            runtime.world.set_time(runtime.world.time + dt)
            await asyncio.sleep(0)

    runtime.forward = forward

    async def sleep(seconds: float):
        sleep_end_time = runtime.world.time + seconds
        while runtime.world.time <= sleep_end_time:
            await asyncio.sleep(0)

    for a in runtime.actors:
        a.sleep = sleep

    yield runtime
    await runtime.stop()
