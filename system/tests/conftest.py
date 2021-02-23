from typing import Generator
from world.world import World
from world.robot import Robot
from world.clock import Clock
import pytest
pytest.register_assert_rewrite("tests.helper")


@pytest.fixture
def world() -> Generator:

    clock = Clock(interval=0.1)
    robot = Robot(width=0.5)
    world = World(clock=clock, robot=robot)
    from tests.helper import set_global_world
    set_global_world(world)
    yield world
