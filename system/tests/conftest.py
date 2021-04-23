from typing import Generator
from world.machine import MockedMachine
from world.world import World
from world.robot import Robot
import pytest
pytest.register_assert_rewrite("tests.helper")


@pytest.fixture
def world() -> Generator:

    machine = MockedMachine()
    robot = Robot(machine=machine)
    world = World(robot=robot)
    from tests.helper import set_global_world
    set_global_world(world)
    yield world
