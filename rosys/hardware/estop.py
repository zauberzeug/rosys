import abc

from ..event import Event
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain


class EStop(Module, abc.ABC):
    """A module that detects when the e-stop is triggered.

    The module has a boolean field `active` that is true when the e-stop is triggered.

    There is also a boolean field `is_soft_estop_active` that is true when the soft e-stop is active.
    It can be set to true or false by calling `set_soft_estop(active: bool)`.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.ESTOP_TRIGGERED = Event()
        """the e-stop was triggered"""

        self.active: bool = False
        self.is_soft_estop_active: bool = False

    @abc.abstractmethod
    async def set_soft_estop(self, active: bool) -> None:
        if active:
            self.ESTOP_TRIGGERED.emit()
        self.is_soft_estop_active = active


class EStopHardware(EStop, ModuleHardware):
    """Hardware implementation of the e-stop module.

    The module expects a dictionary of pin names and pin numbers.
    """

    def __init__(self, robot_brain: RobotBrain, *, name: str = 'estop', pins: dict[str, int]) -> None:
        self.name = name
        self.pins = pins
        lizard_code = '\n'.join(f'{name}_{pin} = Input({number})' for pin, number in pins.items())
        core_message_fields = [f'{name}_{pin}.level' for pin in pins]
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def set_soft_estop(self, active: bool) -> None:
        await super().set_soft_estop(active)
        await self.robot_brain.send(f'en3.level({"false" if active else "true"})')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        active = any(int(words.pop(0)) == 0 for _ in self.pins)
        if active and not self.active:
            self.ESTOP_TRIGGERED.emit()
        self.active = active


class EStopSimulation(EStop, ModuleSimulation):
    """Simulation of the e-stop module."""

    def activate(self) -> None:
        if not self.active:
            self.ESTOP_TRIGGERED.emit()
        self.active = True

    def deactivate(self) -> None:
        self.active = False

    async def set_soft_estop(self, active: bool) -> None:
        await super().set_soft_estop(active)
