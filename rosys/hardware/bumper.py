import abc
from typing import Optional

from ..event import Event
from .estop import EStop
from .expander import ExpanderHardware
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain


class Bumper(Module, abc.ABC):
    """A module that detects when a bumper is triggered."""

    def __init__(self, estop: Optional[EStop], **kwargs) -> None:
        self.estop = estop
        super().__init__(**kwargs)

        self.BUMPER_TRIGGERED = Event()
        """a bumper was triggered (argument: the bumper name)"""

        self.active_bumpers: list[str] = []


class BumperHardware(Bumper, ModuleHardware):
    """Hardware implementation of the bumper module.

    The module expects a dictionary of pin names and pin numbers.
    If an e-stop is provided, the module will not trigger bumpers if the e-stop is active.
    """

    def __init__(self, robot_brain: RobotBrain, *,
                 expander: Optional[ExpanderHardware] = None,
                 name: str = 'bumper',
                 pins: dict[str, int],
                 estop: Optional[EStop] = None) -> None:
        self.name = name
        self.pins = pins
        lizard_code = '\n'.join(f'{name}_{pin} = {expander.name + "." if expander else ""}Input({number})'
                                for pin, number in pins.items())
        core_message_fields = [f'{name}_{pin}.level' for pin in pins]
        super().__init__(robot_brain=robot_brain,
                         lizard_code=lizard_code,
                         core_message_fields=core_message_fields,
                         estop=estop)

    def handle_core_output(self, time: float, words: list[str]) -> None:
        if self.estop and self.estop.active:
            return
        active_bumpers = [pin for pin in self.pins if int(words.pop(0)) == 1]
        for pin in active_bumpers:
            if pin not in self.active_bumpers:
                self.BUMPER_TRIGGERED.emit(pin)
        self.active_bumpers[:] = active_bumpers


class BumperSimulation(Bumper, ModuleSimulation):
    """Simulation of the bumper module."""

    def set_active(self, pin: str, active: bool) -> None:
        if active and pin not in self.active_bumpers:
            self.BUMPER_TRIGGERED.emit(pin)
            self.active_bumpers.append(pin)
        if not active and pin in self.active_bumpers:
            self.active_bumpers.remove(pin)
