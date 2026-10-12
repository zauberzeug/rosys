import pytest
from test_robot_brain import CommunicationSimulation  # pylint: disable=wrong-import-order

from rosys.hardware import (
    CanHardware,
    JoystickHardware,
    JoystickSimulation,
    RobotBrain,
    WheelsHardware,
    WheelsSimulation,
)


@pytest.fixture
def robot_brain() -> RobotBrain:
    return RobotBrain(CommunicationSimulation())


def test_joystick_drives_the_named_wheels(robot_brain: RobotBrain) -> None:
    wheels = WheelsHardware(robot_brain, can=CanHardware(robot_brain), name='tracks')
    joystick = JoystickHardware(robot_brain, wheels=wheels, max_linear_speed=0.6, max_angular_speed=1.2,
                                ramp=1.5, turn_reduction=0.4, timeout=0.8)

    assert joystick.lizard_code.splitlines() == [
        'joystick = Joystick(tracks)',
        'joystick.max_linear_speed = 0.6',
        'joystick.max_angular_speed = 1.2',
        'joystick.ramp = 1.5',
        'joystick.turn_reduction = 0.4',
        'joystick.timeout = 0.8',
    ]
    assert joystick.core_message_fields == ['joystick.active']


@pytest.mark.parametrize('kwargs', [
    {},
    {'max_linear_speed': -1.0},
    {'max_linear_speed': 1.0, 'ramp': -1.0},
    {'max_linear_speed': 1.0, 'timeout': 0.0},
])
def test_joystick_rejects_settings_lizard_would_refuse(robot_brain: RobotBrain, kwargs: dict) -> None:
    wheels = WheelsHardware(robot_brain, can=CanHardware(robot_brain))

    with pytest.raises(ValueError):
        JoystickHardware(robot_brain, wheels=wheels, **kwargs)


def test_joystick_needs_wheels_that_are_a_lizard_module(robot_brain: RobotBrain) -> None:
    with pytest.raises(ValueError):
        JoystickHardware(robot_brain, wheels=WheelsSimulation(), max_linear_speed=1.0)


def test_joystick_reports_when_it_moves_the_wheels(robot_brain: RobotBrain) -> None:
    wheels = WheelsHardware(robot_brain, can=CanHardware(robot_brain))
    joystick = JoystickHardware(robot_brain, wheels=wheels, max_linear_speed=1.0)
    events: list[str] = []
    joystick.ACTIVATED.subscribe(lambda: events.append('activated'))
    joystick.DEACTIVATED.subscribe(lambda: events.append('deactivated'))

    for word in ['false', 'true', 'true', 'false']:
        joystick.handle_core_output(0.0, [word])

    assert events == ['activated', 'deactivated']
    assert joystick.is_active is False


def test_simulated_joystick_can_pretend_a_remote() -> None:
    joystick = JoystickSimulation()
    joystick.set_active(True)

    assert joystick.is_active is True
