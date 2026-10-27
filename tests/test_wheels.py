import math

import pytest
from test_robot_brain import CommunicationSimulation, connect  # pylint: disable=wrong-import-order

from rosys.geometry import Velocity
from rosys.hardware import CanHardware, RobotBrain, RobotHardware, WheelsHardware, WheelsSimulation, cap_wheel_speed
from rosys.testing import forward

WIDTH = 0.5
MAX_WHEEL_SPEED = 0.6


@pytest.mark.parametrize(('linear', 'angular', 'expected'), [
    (1.0, -2.0, (0.4, -0.8)),  # curve: outer wheel 1.0 + 2.0 * 0.25 = 1.5 m/s
    (-1.0, 0.0, (-0.6, 0.0)),  # straight
    (0.0, 4.0, (0.0, 2.4)),  # on the spot
    (0.3, 1.2, (0.3, 1.2)),  # exactly at the limit
    (0.2, 0.4, (0.2, 0.4)),  # below the limit
])
def test_cap_wheel_speed(linear: float, angular: float, expected: tuple[float, float]) -> None:
    """The faster wheel is scaled down to the limit, keeping the curvature and the directions."""
    assert cap_wheel_speed(linear, angular, width=WIDTH, max_wheel_speed=MAX_WHEEL_SPEED) == pytest.approx(expected)


@pytest.mark.parametrize(('linear', 'angular', 'max_wheel_speed'), [
    (1.0, 2.0, None),
    (math.inf, 0.0, MAX_WHEEL_SPEED),
])
def test_cap_wheel_speed_leaves_command_unchanged(linear: float, angular: float, max_wheel_speed: float | None) -> None:
    """Without a limit, or with a non-finite command, the command passes unchanged."""
    assert cap_wheel_speed(linear, angular, width=WIDTH, max_wheel_speed=max_wheel_speed) == (linear, angular)


async def test_simulation_drives_and_reports_the_capped_command(rosys_integration: None) -> None:
    """A curve above the limit is commanded and driven at the capped speed."""
    wheels = WheelsSimulation(WIDTH, max_wheel_speed=MAX_WHEEL_SPEED)
    commanded: list[Velocity] = []
    wheels.VELOCITY_COMMANDED.subscribe(commanded.append)
    await wheels.drive(1.0, 2.0)
    await forward(seconds=0.1)
    assert (commanded[-1].linear, commanded[-1].angular) == pytest.approx((0.4, 0.8))
    assert (wheels.linear_target_speed, wheels.angular_target_speed) == pytest.approx((0.4, 0.8))
    assert (wheels.linear_velocity, wheels.angular_velocity) == pytest.approx((0.4, 0.8))


async def test_simulation_without_limit_drives_unchanged(rosys_integration: None) -> None:
    """Without a limit a fast curve is driven as requested."""
    wheels = WheelsSimulation(WIDTH)
    await wheels.drive(1.0, 2.0)
    await forward(seconds=0.1)
    assert (wheels.linear_velocity, wheels.angular_velocity) == (1.0, 2.0)


async def test_hardware_sends_the_capped_command(rosys_integration: None) -> None:
    """The robot brain receives the capped speed, and the startup script sets the limit as a safety net."""
    communication = CommunicationSimulation()
    robot_brain = RobotBrain(communication, enable_esp_on_startup=False)
    can = CanHardware(robot_brain)
    wheels = WheelsHardware(robot_brain, can=can, width=WIDTH, max_wheel_speed=MAX_WHEEL_SPEED)
    robot = RobotHardware([can, wheels], robot_brain)
    assert f'wheels.max_wheel_speed = {MAX_WHEEL_SPEED}' in robot.generate_lizard_code()
    await connect(communication)

    await wheels.drive(1.0, 2.0)
    speed_commands = [line for line in communication.sent if line.startswith('wheels.speed(')]
    linear, angular = map(float, speed_commands[-1].removeprefix('wheels.speed(').removesuffix(')').split(','))
    assert (linear, angular) == pytest.approx((0.4, 0.8))


async def test_hardware_without_limit_leaves_the_startup_script_unchanged(rosys_integration: None) -> None:
    """Without a limit the startup script does not mention it, so it runs on any Lizard firmware."""
    robot_brain = RobotBrain(CommunicationSimulation(), enable_esp_on_startup=False)
    can = CanHardware(robot_brain)
    wheels = WheelsHardware(robot_brain, can=can)
    assert 'max_wheel_speed' not in RobotHardware([can, wheels], robot_brain).generate_lizard_code()


@pytest.mark.parametrize('max_wheel_speed', [0.0, -0.5, math.inf, math.nan, True])
async def test_invalid_max_wheel_speed_is_rejected(max_wheel_speed: float, rosys_integration: None) -> None:
    """Limits Lizard would read as "off" or reject fail fast instead of aborting the Lizard startup script."""
    with pytest.raises(ValueError):
        WheelsSimulation(max_wheel_speed=max_wheel_speed)
