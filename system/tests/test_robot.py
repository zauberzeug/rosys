from runtime import Runtime
import pytest
import numpy as np
from numpy import rad2deg as deg
from tests.helper import assert_pose, drive, power, automate, condition
import asyncio
from runtime import Runtime


@pytest.mark.asyncio
async def test_drive(runtime: Runtime):

    assert_pose(0, 0, deg=0)

    await runtime.run(seconds=1.0)
    assert_pose(0, 0, deg=0)

    await drive(1.0)
    await runtime.run(seconds=1.0)
    assert_pose(1.0, 0, deg=0)

    await drive(0.0, deg=90)
    await runtime.run(seconds=0.5)
    assert_pose(1.0, 0, deg=45)

    await drive(1.0)
    await runtime.run(seconds=np.sqrt(2))
    assert_pose(2.0, 1.0, deg=45, linear_tolerance=0.1)


@pytest.mark.asyncio
async def test_driving_an_arc(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    async def arc():
        while runtime.world.robot.pose.x < 2:
            await drive(1, deg=26)
            await asyncio.sleep(0)
        await drive(0, deg=0)

    automate(arc())

    await runtime.run(seconds=5.0)
    assert_pose(2, 1.3, deg=65)


@pytest.mark.asyncio
async def test_driving_a_square(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    async def square():
        await drive(1, deg=0)
        await condition(lambda r: r.pose.x >= 2)
        await drive(0, deg=90)
        await condition(lambda r: deg(r.pose.yaw) >= 90)
        await drive(1, deg=0)
        await condition(lambda r: r.pose.y >= 2)
        await drive(0, deg=90)
        await condition(lambda r: deg(r.pose.yaw) >= 180)
        await drive(1, deg=0)
        await condition(lambda r: r.pose.x <= 0)
        await drive(0, deg=90)
        await condition(lambda r: deg(r.pose.yaw) >= 270)
        await drive(1, deg=0)
        await condition(lambda r: r.pose.y <= 0)
        await drive(0, deg=0)

    automate(square())
    await runtime.run(seconds=5.0)
    assert_pose(2, 2, deg=90)
    await runtime.run(seconds=6.0)
    assert_pose(0, 0, deg=270)


@pytest.mark.asyncio
async def test_power(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    await power(1, 1)
    await runtime.run(seconds=1.0)
    assert_pose(1, 0, deg=0)
