import pytest

from rosys.automation import Automator
from rosys.driving import Driver
from rosys.geometry import Pose, Spline
from rosys.hardware import Robot
from rosys.test import assert_pose, forward
import rosys


async def test_driving_an_arc(driver: Driver, automator: Automator, robot: Robot):
    assert_pose(0, 0, deg=0)
    automator.start(driver.drive_arc())
    await forward(x=2)
    assert_pose(2, 1, deg=56)


async def test_driving_a_square(driver: Driver, automator: Automator, robot: Robot):
    assert_pose(0, 0, deg=0)
    automator.start(driver.drive_square())
    await forward(x=1)
    assert_pose(1, 0, deg=0)
    await forward(y=1)
    assert_pose(1, 1, deg=90, deg_tolerance=3)
    await forward(x=0)
    assert_pose(0, 1, deg=180, deg_tolerance=3)
    await forward(y=0)
    assert_pose(0, 0, deg=270, deg_tolerance=3)


async def test_pause_and_resume_spline(driver: Driver, automator: Automator, robot: Robot):
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


async def test_pause_and_resume_square(driver: Driver, automator: Automator, robot: Robot):
    automator.start(driver.drive_square())
    await forward(x=1)
    assert_pose(1, 0, deg=0)

    automator.pause(because='test')
    await forward(seconds=1)
    assert_pose(1, 0, deg=0)

    automator.resume()
    await forward(y=1)
    assert_pose(1, 1, deg=90, deg_tolerance=3)


@pytest.mark.parametrize('dx', [2, 10])
async def test_driving_a_spline(driver: Driver, automator: Automator, robot: Robot, dx: float):
    assert_pose(0, 0, deg=0)
    spline = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=dx, y=1, yaw=0))
    automator.start(driver.drive_spline(spline))
    await forward(x=dx)
    assert_pose(dx, 1, deg=0, deg_tolerance=5)


async def test_aborting_a_drive(driver: Driver, automator: Automator, robot: Robot):
    assert_pose(0, 0, deg=0)
    automator.start(driver.drive_spline(Spline.from_poses(Pose(x=0), Pose(x=2))))
    cause: list[str] = []
    automator.AUTOMATION_STOPPED.register(cause.append)
    await forward(x=1)
    assert_pose(1, 0, deg=0)
    driver.abort()
    await forward(seconds=1)
    assert_pose(1, 0, deg=0)
    assert cause == ['an exception occurred in an automation']


async def test_finally_block(automator: Automator):
    events: list[str] = []

    async def run() -> None:
        try:
            while True:
                events.append('tick')
                await rosys.sleep(3)
        finally:
            events.append('tock')

    automator.start(run())
    await forward(seconds=10)
    automator.stop(because='test')
    await forward(seconds=1)
    assert events == ['tick', 'tick', 'tick', 'tick', 'tock']
