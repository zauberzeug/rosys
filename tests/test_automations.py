from rosys.automation import Automator
from rosys.driving import Driver
from rosys.geometry import Pose, Spline
from rosys.hardware import Robot
from rosys.test import assert_pose, forward


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


async def test_driving_a_spline(driver: Driver, automator: Automator, robot: Robot):
    assert_pose(0, 0, deg=0)
    spline = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=2, y=1, yaw=0))
    automator.start(driver.drive_spline(spline))
    await forward(x=2)
    assert_pose(2, 1, deg=0, deg_tolerance=5)
