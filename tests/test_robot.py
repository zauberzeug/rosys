import pytest
import numpy as np
from rosys.runtime import Runtime
from rosys.test.helper import assert_pose


@pytest.mark.asyncio
async def test_drive(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    await runtime.forward(seconds=1.0)
    assert_pose(0, 0, deg=0)
    await runtime.esp.drive(1.0, 0.0)
    await runtime.forward(seconds=1.0)
    assert_pose(1.0, 0, deg=0)
    await runtime.esp.drive(0.0, np.deg2rad(90))
    await runtime.forward(seconds=0.5, dt=0.005)
    assert_pose(1.0, 0, deg=45)

    await runtime.esp.drive(1.0, 0.0)
    await runtime.forward(seconds=np.sqrt(2))
    assert_pose(2.0, 1.0, deg=45, linear_tolerance=0.1)


@pytest.mark.asyncio
async def test_power(runtime: Runtime):
    assert_pose(0, 0, deg=0)

    await runtime.esp.power(1, 1)
    await runtime.forward(seconds=1.0)
    assert_pose(1, 0, deg=0)


@pytest.mark.asyncio
async def test_stopping_robot_when_runime_stops(runtime: Runtime):
    await runtime.esp.power(1, 1)
    await runtime.forward(seconds=1.0)
    await runtime.stop()
    assert runtime.esp.linear_velocity == 0
    assert runtime.esp.angular_velocity == 0
