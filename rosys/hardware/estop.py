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

        self.ESTOP_TRIGGERED = Event[[]]()
        """the e-stop was triggered"""
        self.ESTOP_RELEASED = Event[[]]()
        """the e-stop was released"""
        self.active: bool = False
        self.is_soft_estop_active: bool = False
        self.pressed_estops: list[int] = []

    @abc.abstractmethod
    async def set_soft_estop(self, active: bool) -> None:
        if active:
            self.ESTOP_TRIGGERED.emit()
        self.is_soft_estop_active = active


class EStopHardware(EStop, ModuleHardware):
    """Hardware implementation of the e-stop module.

    The module expects a dictionary of pin names and pin numbers.
    """

    def __init__(self, robot_brain: RobotBrain, *, name: str = 'estop', pins: dict[str, int], inverted: bool = True) -> None:
        self.name = name
        self.pins = pins
        self.inverted = inverted
        lizard_code = '\n'.join(f'{name}_{pin} = Input({number})' for pin, number in pins.items())
        if inverted:
            lizard_code += '\n' + '\n'.join(f'{name}_{pin}.inverted = true' for pin in pins)
        core_message_fields = [f'{name}_{pin}.active' for pin in pins]
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def set_soft_estop(self, active: bool) -> None:
        await super().set_soft_estop(active)
        await self.robot_brain.send(f'en3.level({"false" if active else "true"})')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        corelist = [words.pop(0) == 'true' for _ in self.pins]
        active = any(corelist)
        pressed = [index for index, value in enumerate(corelist) if value]
        if pressed != self.pressed_estops:
            self.log.warning('E-Stop %s changed', pressed)
        self.pressed_estops[:] = pressed
        if active and not self.active:
            self.ESTOP_TRIGGERED.emit()
        if self.active and not active:
            self.ESTOP_RELEASED.emit()
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
