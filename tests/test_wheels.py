import math

import pytest
from test_robot_brain import CommunicationSimulation  # pylint: disable=wrong-import-order

from rosys.hardware import CanHardware, RobotBrain, RobotHardware, WheelsHardware, WheelsSimulation
from rosys.testing import forward

WIDTH = 0.5
MAX_WHEEL_SPEED = 0.6


@pytest.mark.parametrize('linear_sign', [1, -1])
@pytest.mark.parametrize('angular_sign', [1, -1])
async def test_simulation_caps_the_faster_wheel_and_keeps_the_curve(linear_sign: int, angular_sign: int,
                                                                    rosys_integration: None) -> None:
    """A curve above the limit drives the outer wheel at the limit, with the requested curvature and directions."""
    wheels = WheelsSimulation(WIDTH, max_wheel_speed=MAX_WHEEL_SPEED)
    linear, angular = linear_sign * 1.0, angular_sign * 2.0
    await wheels.drive(linear, angular)
    await forward(seconds=0.1)
    left = wheels.linear_velocity - wheels.angular_velocity * WIDTH / 2
    right = wheels.linear_velocity + wheels.angular_velocity * WIDTH / 2
    assert max(abs(left), abs(right)) == pytest.approx(MAX_WHEEL_SPEED)
    assert wheels.angular_velocity / wheels.linear_velocity == pytest.approx(angular / linear)
    assert math.copysign(1, wheels.linear_velocity) == linear_sign


async def test_simulation_keeps_commands_below_the_limit(rosys_integration: None) -> None:
    """A command whose faster wheel stays below the limit is driven unchanged."""
    wheels = WheelsSimulation(WIDTH, max_wheel_speed=MAX_WHEEL_SPEED)
    await wheels.drive(0.3, -1.0)
    await forward(seconds=0.1)
    assert wheels.linear_velocity == pytest.approx(0.3)
    assert wheels.angular_velocity == pytest.approx(-1.0)


async def test_simulation_turns_on_the_spot_at_the_limit(rosys_integration: None) -> None:
    """Turning on the spot above the limit spins both wheels at the limit."""
    wheels = WheelsSimulation(WIDTH, max_wheel_speed=MAX_WHEEL_SPEED)
    await wheels.drive(0.0, 4.0)
    await forward(seconds=0.1)
    assert wheels.linear_velocity == 0.0
    assert wheels.angular_velocity == pytest.approx(MAX_WHEEL_SPEED / (WIDTH / 2))


@pytest.mark.parametrize('max_wheel_speed', [0.0, -0.5, math.inf, math.nan])
async def test_invalid_max_wheel_speed_is_rejected(max_wheel_speed: float, rosys_integration: None) -> None:
    """Zero, negative and non-finite limits fail fast instead of aborting the Lizard startup script."""
    with pytest.raises(ValueError):
        WheelsSimulation(max_wheel_speed=max_wheel_speed)
    robot_brain = RobotBrain(CommunicationSimulation(), enable_esp_on_startup=False)
    with pytest.raises(ValueError):
        WheelsHardware(robot_brain, can=CanHardware(robot_brain), max_wheel_speed=max_wheel_speed)


@pytest.mark.parametrize('max_wheel_speed', [None, MAX_WHEEL_SPEED])
async def test_hardware_sets_max_wheel_speed_in_the_startup_script(max_wheel_speed: float | None,
                                                                   rosys_integration: None) -> None:
    """The Lizard startup script sets ``max_wheel_speed`` only when a limit is given."""
    robot_brain = RobotBrain(CommunicationSimulation(), enable_esp_on_startup=False)
    can = CanHardware(robot_brain)
    wheels = WheelsHardware(robot_brain, can=can, max_wheel_speed=max_wheel_speed)
    lizard_code = RobotHardware([can, wheels], robot_brain).generate_lizard_code()
    assert ('wheels.max_wheel_speed = 0.6' in lizard_code) == (max_wheel_speed is not None)
