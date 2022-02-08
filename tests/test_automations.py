import pytest
from rosys.automations import drive_arc, drive_path, drive_to, drive_spline, drive_square
from rosys.world import PathSegment, Point, Pose, Spline
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
    await runtime.forward(seconds=1.0)
    assert_pose(0.5, 0, deg=0)
    await runtime.forward(seconds=6.0)
    assert_pose(1, 0.8, deg=113)
    await runtime.forward(seconds=14.0)
    assert_pose(0, 0, deg=270, linear_tolerance=0.15, deg_tolerance=10.0)


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

    await runtime.forward(seconds=8.1)
    runtime.automator.pause(because='test')
    assert_pose(1, 0.7, deg=144, linear_tolerance=0.2)

    await runtime.forward(seconds=2)
    assert_pose(1, 0.7, deg=144, linear_tolerance=0.2)

    runtime.automator.resume()
    await runtime.forward(seconds=10)
    assert_pose(0, 0, deg=270, linear_tolerance=0.5, deg_tolerance=10.0)


@pytest.mark.asyncio
async def test_driving_a_spline(runtime: TestRuntime):
    assert_pose(0, 0, deg=0)
    s = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=2, y=1, yaw=0))
    runtime.automator.start(drive_spline(s, runtime.world, runtime.hardware))
    await runtime.forward(seconds=20)
    assert_pose(1.9, 1, deg=4.4)
