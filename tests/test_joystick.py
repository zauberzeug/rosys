from rosys.hardware import CanHardware, JoystickHardware, JoystickSimulation, RobotBrain, WheelsHardware
from rosys.hardware.communication import Communication


class SilentCommunication(Communication):

    @classmethod
    def is_possible(cls) -> bool:
        return True

    async def send(self, msg: str) -> None:
        pass

    async def read(self) -> str | None:
        return None


def test_joystick_drives_the_named_wheels() -> None:
    robot_brain = RobotBrain(SilentCommunication())
    can = CanHardware(robot_brain)
    wheels = WheelsHardware(robot_brain, can=can, name='tracks', max_linear_speed=0.6, max_angular_speed=1.2)
    joystick = JoystickHardware(robot_brain, wheels=wheels, ramp=1.5, turn_reduction=0.4, timeout=0.8)

    assert 'tracks.max_linear_speed = 0.6' in wheels.lizard_code
    assert 'tracks.max_angular_speed = 1.2' in wheels.lizard_code
    assert joystick.lizard_code.splitlines() == [
        'joystick = Joystick(tracks)',
        'joystick.ramp = 1.5',
        'joystick.turn_reduction = 0.4',
        'joystick.timeout = 0.8',
    ]
    assert joystick.core_message_fields == ['joystick.active']


def test_wheels_leave_the_maxima_to_lizard_by_default() -> None:
    robot_brain = RobotBrain(SilentCommunication())
    wheels = WheelsHardware(robot_brain, can=CanHardware(robot_brain))

    assert 'max_linear_speed' not in wheels.lizard_code
    assert 'max_angular_speed' not in wheels.lizard_code


def test_joystick_reports_when_a_remote_drives() -> None:
    robot_brain = RobotBrain(SilentCommunication())
    wheels = WheelsHardware(robot_brain, can=CanHardware(robot_brain))
    joystick = JoystickHardware(robot_brain, wheels=wheels)
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
