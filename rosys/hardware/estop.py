import abc

from ..event import Event
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain


class EStop(Module, abc.ABC):
    """A module that detects when the e-stop is triggered."""

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.ESTOP_TRIGGERED = Event()
        """the e-stop was triggered"""

        self.active: bool = False
        self.en3_active: bool = False

    @abc.abstractmethod
    async def software_emergency_stop(self) -> None:
        self.ESTOP_TRIGGERED.emit()
        self.en3_active = True

    @abc.abstractmethod
    async def release_en3(self) -> None:
        self.en3_active = False


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

    async def software_emergency_stop(self) -> None:
        await super().software_emergency_stop()
        await self.robot_brain.send(f'en3.off()')

    async def release_en3(self) -> None:
        await super().release_en3()
        await self.robot_brain.send(f'en3.on()')

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
