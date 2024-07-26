import numpy as np
import pytest

import rosys
from rosys.driving.odometer import Odometer
from rosys.hardware import Robot, Wheels, WheelsSimulation
from rosys.testing import assert_pose, forward


@pytest.mark.usefixtures('odometer')
async def test_drive(wheels: Wheels, robot: Robot) -> None:
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
    assert_pose(2.0, 1.0, deg=45, position_tolerance=0.1)


@pytest.mark.usefixtures('odometer')
async def test_stopping_robot_when_rosys_stops(wheels: WheelsSimulation, robot: Robot) -> None:
    await wheels.drive(1, 0)
    await forward(x=1.0, y=0.0)
    await rosys.shutdown()
    assert wheels.linear_velocity == 0
    assert wheels.angular_velocity == 0


async def test_robot_can_slip_in_simulation(wheels: WheelsSimulation, robot: Robot, odometer: Odometer) -> None:
    wheels.slip_factor_right = 0.3
    await wheels.drive(1, 0)
    await forward(seconds=3.0)
    assert odometer.prediction.x == pytest.approx(3.0, abs=0.1)
    assert odometer.prediction.y == pytest.approx(0.0, abs=0.001)
    assert wheels.pose.x == pytest.approx(1.4, abs=0.1)
    assert wheels.pose.y == pytest.approx(-1.7, abs=0.1)
