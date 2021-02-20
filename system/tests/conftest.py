import pytest
from world.clock import Clock
from world.robot import Robot
from world.world import World
from typing import Generator


@pytest.fixture
def world() -> Generator:
    clock = Clock(interval=0.1)
    robot = Robot(width=0.5)
    world = World(clock=clock, robot=robot)

    yield world
