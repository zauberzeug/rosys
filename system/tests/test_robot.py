import pytest
import numpy as np
from numpy import rad2deg as deg
from tests.helper import assert_pose, drive, power
import asyncio
from world.world import World
from actors.actor import Actor
from actors.clock import TestClock
from actors.serial import MockedSerial
from actors.odometer import Odometer


@pytest.mark.asyncio
async def test_drive(world: World, actors: Array[Actor]):

    assert_pose(0, 0, deg=0)

    await world.run(seconds=1.0)
    assert_pose(0, 0, deg=0)

    await drive(1.0)
    await world.run(seconds=1.0)
    assert_pose(1.0, 0, deg=0)

    await drive(0.0, deg=90)
    await world.run(seconds=0.5)
    assert_pose(1.0, 0, deg=45)

    await drive(1.0)
    await world.run(seconds=np.sqrt(2))
    assert_pose(2.0, 1.0, deg=45, linear_tolerance=0.1)


@pytest.mark.asyncio
async def test_driving_an_arc(world: World):
    assert_pose(0, 0, deg=0)

    async def arc():
        while world.robot.pose.x < 2:
            await drive(1, deg=26)
            await asyncio.sleep(0)
        await drive(0, deg=0)

    world.robot.automate(arc())

    await world.run(seconds=5.0)
    assert_pose(2, 1.3, deg=65)


@pytest.mark.asyncio
async def test_driving_a_square(world: World):
    assert_pose(0, 0, deg=0)

    async def square():
        await drive(1, deg=0)
        await world.robot.condition(lambda r: r.pose.x >= 2)
        await drive(0, deg=90)
        await world.robot.condition(lambda r: deg(r.pose.yaw) >= 90)
        await drive(1, deg=0)
        await world.robot.condition(lambda r: r.pose.y >= 2)
        await drive(0, deg=90)
        await world.robot.condition(lambda r: deg(r.pose.yaw) >= 180)
        await drive(1, deg=0)
        await world.robot.condition(lambda r: r.pose.x <= 0)
        await drive(0, deg=90)
        await world.robot.condition(lambda r: deg(r.pose.yaw) >= 270)
        await drive(1, deg=0)
        await world.robot.condition(lambda r: r.pose.y <= 0)
        await drive(0, deg=0)

    world.robot.automate(square())
    await world.run(seconds=5.0)
    assert_pose(2, 2, deg=90)
    await world.run(seconds=6.0)
    assert_pose(0, 0, deg=270)


@pytest.mark.asyncio
async def test_driving_forward_with_power(world: World):
    assert_pose(0, 0, deg=0)

    await power(1, 1)
    await world.run(seconds=1.0)
    assert_pose(1, 0, deg=0)
