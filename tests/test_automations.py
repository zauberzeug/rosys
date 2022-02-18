import pytest
from rosys.automations import drive_arc, drive_spline, drive_square
from rosys.world import Pose, Spline
from rosys.test import assert_pose, TestRuntime


@pytest.mark.asyncio
async def test_driving_an_arc(runtime: TestRuntime):
    assert_pose(0, 0, deg=0)
    runtime.automator.start(drive_arc(runtime.world, runtime.hardware))
    await runtime.forward(x=2)
    assert_pose(2, 1, deg=56)


@pytest.mark.asyncio
async def test_driving_a_square(runtime: TestRuntime):
    assert_pose(0, 0, deg=0)
    runtime.automator.start(drive_square(runtime.world, runtime.hardware))
    await runtime.forward(x=1)
    assert_pose(1, 0, deg=0)
    await runtime.forward(y=1)
    assert_pose(1, 1, deg=90, deg_tolerance=2)
    await runtime.forward(x=0)
    assert_pose(0, 1, deg=180, deg_tolerance=2)
    await runtime.forward(y=0)
    assert_pose(0, 0, deg=270, deg_tolerance=2)


@pytest.mark.asyncio
async def test_pause_and_resume_spline(runtime: TestRuntime):
    s = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=2, y=0, yaw=0))
    runtime.automator.start(drive_spline(s, runtime.world, runtime.hardware))
    await runtime.forward(x=1)
    assert_pose(1, 0, deg=0)

    runtime.automator.pause(because='test')
    await runtime.forward(seconds=1.0)
    assert_pose(1, 0, deg=0)

    runtime.automator.resume()
    await runtime.forward(x=2)
    assert_pose(2, 0, deg=0)


@pytest.mark.asyncio
async def test_pause_and_resume_square(runtime: TestRuntime):
    runtime.automator.start(drive_square(runtime.world, runtime.hardware))
    await runtime.forward(x=1)
    assert_pose(1, 0, deg=0)

    runtime.automator.pause(because='test')
    await runtime.forward(seconds=1)
    assert_pose(1, 0, deg=0)

    runtime.automator.resume()
    await runtime.forward(y=1)
    assert_pose(1, 1, deg=90, deg_tolerance=2)


@pytest.mark.asyncio
async def test_driving_a_spline(runtime: TestRuntime):
    assert_pose(0, 0, deg=0)
    s = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=2, y=1, yaw=0))
    runtime.automator.start(drive_spline(s, runtime.world, runtime.hardware))
    await runtime.forward(x=2)
    assert_pose(2, 1, deg=0, deg_tolerance=5)
