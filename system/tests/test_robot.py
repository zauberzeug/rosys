from actors.spline_driver import SplineDriver
from actors.square_driver import SquareDriver
from actors.arc_driver import ArcDriver
from actors.esp import Esp
from runtime import Runtime
import pytest
import numpy as np
from tests.helper import assert_pose, drive, power
import asyncio
from runtime import Runtime
from world.world import World
import time


@pytest.mark.asyncio
async def test_drive(runtime: Runtime):

    assert_pose(0, 0, deg=0)

    await runtime.run(seconds=1.0)
    assert_pose(0, 0, deg=0)

    await drive(1.0)
    await runtime.run(seconds=1.0)
    assert_pose(1.0, 0, deg=0)

    await drive(0.0, deg=90)
    await runtime.run(seconds=0.5)
    assert_pose(1.0, 0, deg=45)

    await drive(1.0)
    await runtime.run(seconds=np.sqrt(2))
    assert_pose(2.0, 1.0, deg=45, linear_tolerance=0.1)


@pytest.mark.asyncio
async def test_driving_an_arc(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    runtime.add(ArcDriver)

    await runtime.run(seconds=5.0)
    assert_pose(2, 1.3, deg=62)


@pytest.mark.asyncio
async def test_driving_a_square(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    runtime.add(SquareDriver)
    await runtime.run(seconds=5.1)
    assert_pose(2, 2, deg=90)
    await runtime.run(seconds=7.0)
    assert_pose(0, 0, deg=270)


@pytest.mark.asyncio
async def test_pause_and_resume(runtime: Runtime):
    start = time.time()
    runtime.add(SquareDriver)

    await runtime.run(seconds=5.01)
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
    runtime.add(SplineDriver)
    await runtime.run(seconds=10)
    assert_pose(2, 1, deg=7)


@pytest.mark.asyncio
async def test_power(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    await power(1, 1)
    await runtime.run(seconds=1.0)
    assert_pose(1, 0, deg=0)
