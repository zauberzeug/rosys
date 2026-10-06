import abc

from nicegui import Event

from ..helpers import remove_indentation
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain
from .wheels import Wheels


class Joystick(Module, abc.ABC):
    """A remote control drives the wheels with relative values through Lizard's Joystick module.

    The remote (e.g. the Zauberzeug app via Bluetooth) sends `joystick.drive(forward, turn)` with values in -1..1
    straight to the microcontroller; Lizard ramps them, scales them to the wheels' maximum speeds and drives the wheels.
    This module only creates the Lizard code and reports whether a remote is currently driving,
    so an automation can step aside while someone steers by hand.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.ACTIVATED = Event[[]]()
        """a remote started driving the wheels"""
        self.DEACTIVATED = Event[[]]()
        """the remote released the stick or went silent"""

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

    Needs a Lizard firmware with the Joystick module.
    Keep the name `joystick`: the Zauberzeug app looks for a module of that name when it connects
    and drives robots without it the old way, with `wheels.speed(...)`.
    The wheels' maximum speeds come from their `max_linear_speed`/`max_angular_speed` or, if those are 0,
    from the drivetrain's own speed limit (see https://lizard.dev/module_reference/#joystick).
    """

    def __init__(self, robot_brain: RobotBrain, *,
                 wheels: Wheels,
                 name: str = 'joystick',
                 ramp: float = 2.0,
                 turn_reduction: float = 0.5,
                 timeout: float = 1.0) -> None:
        """
        :param robot_brain: The robot brain instance to communicate with
        :param wheels: The wheels to drive; their Lizard module name must be the wheels' ``name``
        :param name: The Lizard module name (default: 'joystick', which the Zauberzeug app expects)
        :param ramp: Maximum change of the relative setpoints per second (0 = no ramp)
        :param turn_reduction: Share of the turn rate left at full forward speed (0..1, 1 = no reduction)
        :param timeout: Stop when no drive command arrives for this long (s, 0 = off)
        """
        self.name = name
        wheels_name = getattr(wheels, 'name', 'wheels')
        lizard_code = remove_indentation(f'''
            {name} = Joystick({wheels_name})
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
