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
        self._soft_estop_active: bool = False
        self.pressed_estops: list[int] = []

    @property
    def active(self) -> bool:
        return any(self.pressed_estops) or self._soft_estop_active

    @property
    def is_soft_estop_active(self) -> bool:
        return self._soft_estop_active

    async def set_soft_estop(self, active: bool) -> None:
        self._emit_events(active)
        self._soft_estop_active = active

    def _emit_events(self, value: bool) -> None:
        if value and not self.active:
            self.ESTOP_TRIGGERED.emit()
        if not value and self.active:
            self.ESTOP_RELEASED.emit()


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
        pressed = [index for index, value in enumerate(corelist) if value]
        if pressed != self.pressed_estops:
            self.log.warning('E-Stop %s changed', pressed)
        self._emit_events(any(pressed) or self.is_soft_estop_active)
        self.pressed_estops[:] = pressed


class EStopSimulation(EStop, ModuleSimulation):
    """Simulation of the e-stop module."""

    async def activate(self) -> None:
        await self.set_soft_estop(True)

    async def deactivate(self) -> None:
        await self.set_soft_estop(False)
