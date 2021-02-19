import pytest
import numpy as np
from world.clock import Clock
from world.robot import Robot
from world.world import World
from tests.helper import assert_pose


@pytest.mark.asyncio
async def test_drive():

    clock = Clock(interval=0.1)
    robot = Robot(width=0.5)
    world = World(clock=clock, robot=robot)
    assert_pose(world.robot.pose, 0, 0, 0)

    await world.simulate(seconds=1.0)
    assert_pose(world.robot.pose, 0, 0, 0)

    robot.drive(1.0, 0.0)
    await world.simulate(seconds=1.0)
    assert_pose(world.robot.pose, 1.0, 0, 0)

    robot.drive(0.0, np.deg2rad(90))
    await world.simulate(seconds=0.5)
    assert_pose(world.robot.pose, 1.0, 0, np.deg2rad(45))

    robot.drive(1.0, 0.0)
    await world.simulate(seconds=np.sqrt(2))
    assert_pose(world.robot.pose, 2.0, 1.0, np.deg2rad(45), linear_tolerance=0.1)
