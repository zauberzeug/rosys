import pytest
import numpy as np
from rosys.test import assert_pose, TestRuntime


@pytest.mark.asyncio
async def test_drive(runtime: TestRuntime):
    assert_pose(0, 0, deg=0)

    await runtime.forward(seconds=1.0)
    assert_pose(0, 0, deg=0)
    await runtime.hardware.drive(1.0, 0.0)
    await runtime.forward(seconds=1.0)
    assert_pose(1.0, 0, deg=0)
    await runtime.hardware.drive(0.0, np.deg2rad(90))
    await runtime.forward(seconds=0.5, dt=0.005)
    assert_pose(1.0, 0, deg=45)

    await runtime.hardware.drive(1.0, 0.0)
    await runtime.forward(seconds=np.sqrt(2))
    assert_pose(2.0, 1.0, deg=45, linear_tolerance=0.1)


@pytest.mark.asyncio
async def test_stopping_robot_when_runtime_stops(runtime: TestRuntime):
    await runtime.hardware.drive(1, 0)
    await runtime.forward(seconds=1.0)
    await runtime.shutdown()
    assert runtime.hardware.simulation.linear_velocity == 0
    assert runtime.hardware.simulation.angular_velocity == 0
