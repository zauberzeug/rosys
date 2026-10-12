import abc

from nicegui import Event

from ..helpers import remove_indentation
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain
from .wheels import Wheels


class Joystick(Module, abc.ABC):
    """A remote control drives the wheels with relative values through Lizard's Joystick module.

    The remote (e.g. the Zauberzeug app via Bluetooth) sends `joystick.drive(forward, turn)` with values in -1..1
    straight to the microcontroller; Lizard ramps them, scales them to the joystick's maximum speeds and drives the wheels.
    This module only creates the Lizard code and reports whether the joystick is currently moving the wheels.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.ACTIVATED = Event[[]]()
        """the joystick started moving the wheels (non-zero setpoints)"""
        self.DEACTIVATED = Event[[]]()
        """the joystick stopped moving the wheels: stick released, remote silent or wheels disabled or locked"""

        self.is_active: bool = False

    def _set_active(self, active: bool) -> None:
        if active == self.is_active:
            return
        self.is_active = active
        if active:
            self.ACTIVATED.emit()
        else:
            self.DEACTIVATED.emit()


class JoystickHardware(Joystick, ModuleHardware):
    """Hardware implementation of the joystick module.

    Needs a Lizard firmware with the Joystick module (see https://lizard.dev/module_reference/#joystick).
    Put it after its wheels in the robot's modules, since the Lizard code refers to them.
    Keep the name `joystick`: the Zauberzeug app looks for a module of that name when it connects
    and drives robots without it the old way, with `wheels.speed(...)`.
    After `DEACTIVATED` Lizard repeats the stop for a moment, but falls silent as soon as anyone else drives the wheels,
    so an automation can take over right away.
    """

    def __init__(self, robot_brain: RobotBrain, *,
                 wheels: Wheels,
                 max_linear_speed: float = 0.0,
                 max_angular_speed: float = 0.0,
                 name: str = 'joystick',
                 ramp: float = 2.0,
                 turn_reduction: float = 0.5,
                 timeout: float = 1.0) -> None:
        """
        :param robot_brain: The robot brain instance to communicate with
        :param wheels: The wheels to drive, a Lizard module on the same microcontroller
        :param max_linear_speed: Forward speed at full deflection (m/s, 0 = from ``max_angular_speed``)
        :param max_angular_speed: Turning speed at full deflection (rad/s, 0 = from ``max_linear_speed``)
        :param name: The Lizard module name (default: 'joystick', which the Zauberzeug app expects)
        :param ramp: Maximum change of the relative setpoints per second (0 = no ramp)
        :param turn_reduction: Share of the turn rate left at full forward speed (clamped to 0..1, 1 = no reduction)
        :param timeout: Stop when no drive command arrives for this long (s, must be positive)
        """
        if not isinstance(wheels, ModuleHardware) or not hasattr(wheels, 'name'):
            raise ValueError('the joystick needs wheels that are a Lizard module with a name')
        if max_linear_speed < 0 or max_angular_speed < 0 or max_linear_speed == max_angular_speed == 0:
            raise ValueError('the joystick needs a positive max_linear_speed or max_angular_speed')
        if ramp < 0:
            raise ValueError('the joystick ramp must not be negative')
        if timeout <= 0:
            raise ValueError('the joystick timeout must be positive')
        self.name = name
        lizard_code = remove_indentation(f'''
            {name} = Joystick({wheels.name})
            {name}.max_linear_speed = {max_linear_speed}
            {name}.max_angular_speed = {max_angular_speed}
            {name}.ramp = {ramp}
            {name}.turn_reduction = {turn_reduction}
            {name}.timeout = {timeout}
        ''')
        core_message_fields = [f'{name}.active']
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self._set_active(words.pop(0) == 'true')


class JoystickSimulation(Joystick, ModuleSimulation):
    """Simulation of the joystick module.

    No remote can connect to a simulated robot; `set_active` lets tests pretend one is driving.
    """

    def set_active(self, active: bool) -> None:
        self._set_active(active)
