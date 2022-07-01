import pytest
from rosys.actors import Automator, Driver
from rosys.test import assert_pose, forward
from rosys.world import Pose, Spline


@pytest.mark.asyncio
async def test_driving_an_arc(driver: Driver, automator: Automator):
    assert_pose(0, 0, deg=0)
    automator.start(driver.drive_arc())
    await forward(x=2)
    assert_pose(2, 1, deg=56)


@pytest.mark.asyncio
async def test_driving_a_square(driver: Driver, automator: Automator):
    assert_pose(0, 0, deg=0)
    automator.start(driver.drive_square())
    await forward(x=1)
    assert_pose(1, 0, deg=0)
    await forward(y=1)
    assert_pose(1, 1, deg=90, deg_tolerance=2)
    await forward(x=0)
    assert_pose(0, 1, deg=180, deg_tolerance=2)
    await forward(y=0)
    assert_pose(0, 0, deg=270, deg_tolerance=2)


@pytest.mark.asyncio
async def test_pause_and_resume_spline(driver: Driver, automator: Automator):
    spline = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=2, y=0, yaw=0))
    automator.start(driver.drive_spline(spline))
    await forward(x=1)
    assert_pose(1, 0, deg=0)

    automator.pause(because='test')
    await forward(seconds=1.0)
    assert_pose(1, 0, deg=0)

    automator.resume()
    await forward(x=2)
    assert_pose(2, 0, deg=0)


@pytest.mark.asyncio
async def test_pause_and_resume_square(driver: Driver, automator: Automator):
    automator.start(driver.drive_square())
    await forward(x=1)
    assert_pose(1, 0, deg=0)

    automator.pause(because='test')
    await forward(seconds=1)
    assert_pose(1, 0, deg=0)

    automator.resume()
    await forward(y=1)
    assert_pose(1, 1, deg=90, deg_tolerance=2)


@pytest.mark.asyncio
async def test_driving_a_spline(driver: Driver, automator: Automator):
    assert_pose(0, 0, deg=0)
    spline = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=2, y=1, yaw=0))
    automator.start(driver.drive_spline(spline))
    await forward(x=2)
    assert_pose(2, 1, deg=0, deg_tolerance=5)
