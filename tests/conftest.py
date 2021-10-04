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
        robot=Robot(marker=Marker.four_points(0.24, 0.26, 0.41)),
    ))

    from tests.helper import set_global_runtime
    set_global_runtime(runtime)

    yield runtime
