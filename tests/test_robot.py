import numpy as np
import pytest
from rosys import runtime
from rosys.hardware import Wheels, WheelsSimulation
from rosys.test import assert_pose, forward


@pytest.mark.asyncio
async def test_drive(wheels: Wheels) -> None:
    assert_pose(0, 0, deg=0)

    await forward(seconds=1.0)
    assert_pose(0, 0, deg=0)

    await wheels.drive(1.0, 0.0)
    await forward(seconds=1.0)
    assert_pose(1.0, 0, deg=0)

    await wheels.drive(0.0, np.deg2rad(90))
    await forward(seconds=0.5, dt=0.005)
    assert_pose(1.0, 0, deg=45)

    await wheels.drive(1.0, 0.0)
    await forward(seconds=np.sqrt(2))
    assert_pose(2.0, 1.0, deg=45, linear_tolerance=0.1)


@pytest.mark.asyncio
async def test_stopping_robot_when_runtime_stops(wheels: Wheels) -> None:
    await wheels.drive(1, 0)
    await forward(x=1.0, y=0.0)
    await runtime.shutdown()
    assert isinstance(wheels, WheelsSimulation)
    assert wheels.linear_velocity == 0
    assert wheels.angular_velocity == 0
