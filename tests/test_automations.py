import pytest
from rosys.runtime import Runtime
from rosys.automations.arc import drive_arc
from rosys.automations.square import drive_square
from rosys.automations.spline import drive_spline
from rosys.world.pose import Pose
from rosys.world.spline import Spline
from rosys.test.helper import assert_pose


@pytest.mark.asyncio
async def test_driving_an_arc(runtime: Runtime):
    assert_pose(0, 0, deg=0)
    runtime.automator.add(drive_arc(runtime.world, runtime.esp))
    await runtime.forward(seconds=5.0)
    assert_pose(2, 1.2, deg=62)
    assert len(runtime.notifications) == 1
    assert runtime.notifications[0] == 'pausing automations because the last one has completed'


@pytest.mark.asyncio
async def test_driving_a_square(runtime: Runtime):
    assert_pose(0, 0, deg=0)
    runtime.automator.add(drive_square(runtime.world, runtime.esp))
    await runtime.forward(seconds=1.0)
    assert_pose(0.5, 0, deg=0)
    await runtime.forward(seconds=6.0)
    assert_pose(1, 0.8, deg=105)
    await runtime.forward(seconds=14.0)
    assert_pose(0, 0, deg=270, linear_tolerance=0.15, deg_tolerance=10.0)


@pytest.mark.asyncio
async def test_pause_and_resume_spline(runtime: Runtime):
    start = runtime.world.time
    s = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=2, y=0, yaw=0))
    runtime.automator.add(drive_spline(s, runtime.world, runtime.esp))

    await runtime.forward(seconds=2)
    await runtime.pause()
    assert_pose(1, 0, deg=0)
    assert runtime.world.time == pytest.approx(start + 2, abs=0.1)

    await runtime.forward(seconds=2)
    assert_pose(1, 0, deg=0)
    assert runtime.world.time == pytest.approx(start + 4, abs=0.1)

    runtime.resume()
    await runtime.forward(seconds=1.0)
    assert_pose(1.5, 0, deg=0)
    await runtime.forward(seconds=1.0)
    assert_pose(1.9, 0, deg=0)
    assert runtime.world.time == pytest.approx(start + 6.1, abs=0.1)


@pytest.mark.asyncio
async def test_pause_and_resume_square(runtime: Runtime):
    runtime.automator.add(drive_square(runtime.world, runtime.esp))

    await runtime.forward(seconds=8.1)
    await runtime.pause()
    assert_pose(1, 0.7, deg=136, linear_tolerance=0.2)

    await runtime.forward(seconds=2)
    assert_pose(1, 0.7, deg=136, linear_tolerance=0.2)

    runtime.resume()
    await runtime.forward(seconds=10)
    assert_pose(0, 0, deg=270, linear_tolerance=0.5, deg_tolerance=10.0)


@pytest.mark.asyncio
async def test_driving_a_spline(runtime: Runtime):
    assert_pose(0, 0, deg=0)
    s = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=2, y=1, yaw=0))
    runtime.automator.add(drive_spline(s, runtime.world, runtime.esp))
    await runtime.forward(seconds=20)
    assert_pose(1.9, 1, deg=2.6)
