import pytest
from rosys.automations import drive_arc, drive_spline, drive_square
from rosys.world import Pose, Spline
from rosys.test import assert_pose, TestRuntime


@pytest.mark.asyncio
async def test_driving_an_arc(runtime: TestRuntime):
    assert_pose(0, 0, deg=0)
    runtime.automator.start(drive_arc(runtime.world, runtime.hardware))
    await runtime.forward(seconds=5.0)
    assert_pose(2, 1.2, deg=61)


@pytest.mark.asyncio
async def test_driving_a_square(runtime: TestRuntime):
    assert_pose(0, 0, deg=0)
    runtime.automator.start(drive_square(runtime.world, runtime.hardware))
    await runtime.forward(until=1.9)
    assert_pose(1, 0, deg=0)
    await runtime.forward(until=6.2)
    assert_pose(1, 1, deg=90, deg_tolerance=5)
    await runtime.forward(until=10.5)
    assert_pose(0, 1, deg=180, deg_tolerance=10)
    await runtime.forward(until=14.8)
    assert_pose(0, 0, deg=270, deg_tolerance=10)


@pytest.mark.asyncio
async def test_pause_and_resume_spline(runtime: TestRuntime):
    start = runtime.world.time
    s = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=2, y=0, yaw=0))
    runtime.automator.start(drive_spline(s, runtime.world, runtime.hardware))

    await runtime.forward(seconds=2)
    runtime.automator.pause(because='test')
    assert_pose(1, 0, deg=0)
    assert runtime.world.time == pytest.approx(start + 2, abs=0.1)

    await runtime.forward(seconds=2)
    assert_pose(1, 0, deg=0)
    assert runtime.world.time == pytest.approx(start + 4, abs=0.1)

    runtime.automator.resume()
    await runtime.forward(seconds=1.0)
    assert_pose(1.5, 0, deg=0)
    await runtime.forward(seconds=1.0)
    assert_pose(1.9, 0, deg=0)
    assert runtime.world.time == pytest.approx(start + 6.1, abs=0.1)


@pytest.mark.asyncio
async def test_pause_and_resume_square(runtime: TestRuntime):
    runtime.automator.start(drive_square(runtime.world, runtime.hardware))

    await runtime.forward(until=1.9)
    runtime.automator.pause(because='test')
    assert_pose(1, 0, deg=0)

    await runtime.forward(seconds=2)
    assert_pose(1, 0, deg=0)

    runtime.automator.resume()
    await runtime.forward(until=8.2)
    assert_pose(1, 1, deg=90, deg_tolerance=50)


@pytest.mark.asyncio
async def test_driving_a_spline(runtime: TestRuntime):
    assert_pose(0, 0, deg=0)
    s = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=2, y=1, yaw=0))
    runtime.automator.start(drive_spline(s, runtime.world, runtime.hardware))
    await runtime.forward(seconds=20)
    assert_pose(1.9, 1, deg=4.4)
