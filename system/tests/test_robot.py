import pytest
import numpy as np
from tests.helper import assert_pose, drive, deg


@pytest.mark.asyncio
async def test_drive(world):
    assert_pose(0, 0, deg=0)

    await world.simulate(seconds=1.0)
    assert_pose(0, 0, deg=0)

    drive(1.0)
    await world.simulate(seconds=1.0)
    assert_pose(1.0, 0, deg=0)

    drive(0.0, deg=90)
    await world.simulate(seconds=0.5)
    assert_pose(1.0, 0, deg=45)

    drive(1.0)
    await world.simulate(seconds=np.sqrt(2))
    assert_pose(2.0, 1.0, deg=45, linear_tolerance=0.1)


@pytest.mark.asyncio
async def test_driving_a_square(world):
    assert_pose(0, 0, deg=0)

    async def square():
        drive(1, deg=0)
        await world.robot.condition(lambda r: r.pose.x >= 2)
        drive(0, deg=90)
        await world.robot.condition(lambda r: deg(r.pose.yaw) >= 90)
        drive(1, deg=0)
        await world.robot.condition(lambda r: r.pose.y >= 2)
        drive(0, deg=90)
        await world.robot.condition(lambda r: deg(r.pose.yaw) >= 180)
        drive(1, deg=0)
        await world.robot.condition(lambda r: r.pose.x <= 0)
        drive(0, deg=90)
        await world.robot.condition(lambda r: deg(r.pose.yaw) >= 270)
        drive(1, deg=0)
        await world.robot.condition(lambda r: r.pose.y <= 0)
        drive(0, deg=0)

    world.robot.automate(square())
    await world.simulate(seconds=5.0)
    assert_pose(2, 2, deg=90)
    await world.simulate(seconds=6.0)
    assert_pose(0, 0, deg=270)
