import pytest
import numpy as np
from runtime import Runtime
from automations.arc import arc
from automations.square import square
from automations.spline import spline
from tests.helper import assert_pose


@pytest.mark.asyncio
async def test_drive(runtime: Runtime):

    assert_pose(0, 0, deg=0)

    await runtime.run(seconds=1.0)
    assert_pose(0, 0, deg=0)

    await runtime.esp.drive(1.0, 0.0)
    await runtime.run(seconds=1.0)
    assert_pose(1.0, 0, deg=0)

    await runtime.esp.drive(0.0, np.deg2rad(90))
    await runtime.run(seconds=0.5)
    assert_pose(1.0, 0, deg=45)

    await runtime.esp.drive(1.0, 0.0)
    await runtime.run(seconds=np.sqrt(2))
    assert_pose(2.0, 1.0, deg=45, linear_tolerance=0.1)


@pytest.mark.asyncio
async def test_driving_an_arc(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    runtime.automator.add(arc(runtime.world, runtime.esp))

    await runtime.run(seconds=5.0)
    assert_pose(2, 1.3, deg=62)


@pytest.mark.asyncio
async def test_driving_a_square(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    runtime.automator.add(square(runtime.esp, runtime.world))
    await runtime.run(seconds=5.1)
    assert_pose(2, 2, deg=90)
    await runtime.run(seconds=7.0)
    assert_pose(0, 0, deg=270)


@pytest.mark.asyncio
async def test_pause_and_resume(runtime: Runtime):
    start = runtime.world.time
    runtime.automator.add(square(runtime.esp, runtime.world))

    await runtime.run(seconds=5.05)
    runtime.pause()
    assert_pose(2, 2, deg=90)
    assert runtime.world.time == pytest.approx(start + 5, abs=0.1)
    await runtime.run(seconds=2)
    assert_pose(2, 2, deg=90)
    assert runtime.world.time == pytest.approx(start + 7, abs=0.1)

    runtime.resume()
    await runtime.run(seconds=7.0)
    assert_pose(0, 0, deg=270)

    assert runtime.world.time == pytest.approx(start + 14, abs=0.1)


@pytest.mark.asyncio
async def test_driving_a_spline(runtime: Runtime):
    assert_pose(0, 0, deg=0)
    runtime.automator.add(spline(runtime.world, runtime.esp))
    await runtime.run(seconds=10)
    assert_pose(2, 1, deg=7)


@pytest.mark.asyncio
async def test_power(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    await runtime.esp.power(1, 1)
    await runtime.run(seconds=1.0)
    assert_pose(1, 0, deg=0)
