import pytest
import numpy as np
from rosys.runtime import Runtime
from rosys.automations.arc import drive_arc
from rosys.automations.square import drive_square
from rosys.automations.spline import drive_spline
from rosys.world.pose import Pose
from rosys.world.spline import Spline
from tests.helper import assert_pose


@pytest.mark.asyncio
async def test_drive(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    await runtime.advance_time(seconds=1.0)
    assert_pose(0, 0, deg=0)
    await runtime.esp.drive(1.0, 0.0)
    await runtime.advance_time(seconds=1.0)
    assert_pose(1.0, 0, deg=0)
    await runtime.esp.drive(0.0, np.deg2rad(90))
    await runtime.advance_time(seconds=0.5, step_length=0.005)
    assert_pose(1.0, 0, deg=45)

    await runtime.esp.drive(1.0, 0.0)
    await runtime.advance_time(seconds=np.sqrt(2))
    assert_pose(2.0, 1.0, deg=45, linear_tolerance=0.1)


@pytest.mark.asyncio
async def test_driving_an_arc(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    runtime.automator.add(drive_arc(runtime.world, runtime.esp))

    await runtime.advance_time(seconds=5.0)
    assert_pose(2, 1.2, deg=62)


@pytest.mark.asyncio
async def test_driving_a_square(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    runtime.automator.add(drive_square(runtime.world, runtime.esp))
    await runtime.advance_time(seconds=1.0)
    assert_pose(0.5, 0, deg=0)
    await runtime.advance_time(seconds=6.0)
    assert_pose(1, 0.8, deg=105)
    await runtime.advance_time(seconds=14.0)
    assert_pose(0, 0, deg=270, linear_tolerance=0.15, deg_tolerance=10.0)


@pytest.mark.asyncio
async def test_pause_and_resume_spline(runtime: Runtime):
    start = runtime.world.time
    s = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=2, y=0, yaw=0))
    runtime.automator.add(drive_spline(s, runtime.world, runtime.esp))

    await runtime.advance_time(seconds=2)
    await runtime.pause()
    assert_pose(1, 0, deg=0)
    assert runtime.world.time == pytest.approx(start + 2, abs=0.1)

    await runtime.advance_time(seconds=2)
    assert_pose(1, 0, deg=0)
    assert runtime.world.time == pytest.approx(start + 4, abs=0.1)

    runtime.resume()
    await runtime.advance_time(seconds=1.0)
    assert_pose(1.5, 0, deg=0)
    await runtime.advance_time(seconds=1.0)
    assert_pose(1.9, 0, deg=0)
    assert runtime.world.time == pytest.approx(start + 6.1, abs=0.1)


@pytest.mark.asyncio
async def test_pause_and_resume_square(runtime: Runtime):
    runtime.automator.add(drive_square(runtime.world, runtime.esp))

    await runtime.advance_time(seconds=8.1)
    await runtime.pause()
    assert_pose(1, 0.7, deg=136, linear_tolerance=0.2)

    await runtime.advance_time(seconds=2)
    assert_pose(1, 0.7, deg=136, linear_tolerance=0.2)

    runtime.resume()
    await runtime.advance_time(seconds=10)
    assert_pose(0, 0, deg=270, linear_tolerance=0.5, deg_tolerance=10.0)


@pytest.mark.asyncio
async def test_driving_a_spline(runtime: Runtime):
    assert_pose(0, 0, deg=0)
    s = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=2, y=1, yaw=0))
    runtime.automator.add(drive_spline(s, runtime.world, runtime.esp))
    await runtime.advance_time(seconds=20)
    assert_pose(1.9, 1, deg=2.6)


@pytest.mark.asyncio
async def test_power(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    await runtime.esp.power(1, 1)
    await runtime.advance_time(seconds=1.0)
    assert_pose(1, 0, deg=0)
