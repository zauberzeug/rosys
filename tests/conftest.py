from typing import Generator
import pytest
from rosys.runtime import Runtime, Mode
from rosys.world.world import World, WorldState
from rosys.world.robot import Robot
from rosys.world.marker import Marker

import icecream
icecream.install()

pytest.register_assert_rewrite("tests.helper")


@pytest.fixture
def runtime() -> Generator:
    runtime = Runtime(World(
        mode=Mode.TEST,
        state=WorldState.RUNNING,
        robot=Robot(marker=Marker(points={'front': (0.12, 0), 'back': (-0.12, 0)}, height=0.58)),
    ))

    from tests.helper import set_global_runtime
    set_global_runtime(runtime)

    yield runtime
