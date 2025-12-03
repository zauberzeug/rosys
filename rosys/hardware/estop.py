import abc

from nicegui import Event

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

        self.ESTOP_TRIGGERED = Event[str]()
        """the e-stop was triggered (argument: the e-stop name)"""
        self.ESTOP_RELEASED = Event[str]()
        """the e-stop was released (argument: the e-stop name)"""
        self.active_estops: set[str] = set()

    @property
    def active(self) -> bool:
        """Whether any hardware e-stop or the soft e-stop is active."""
        return any(self.active_estops)

    @property
    def is_soft_estop_active(self) -> bool:
        """Whether the soft e-stop is active."""
        return 'soft' in self.active_estops

    async def set_soft_estop(self, active: bool) -> None:
        """Set the soft e-stop to the given state."""
        if active and not self.is_soft_estop_active:
            self.active_estops.add('soft')
            self.ESTOP_TRIGGERED.emit('soft')
        elif not active and self.is_soft_estop_active:
            self.active_estops.discard('soft')
            self.ESTOP_RELEASED.emit('soft')


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
        previous_active_estops = self.active_estops.copy()
        self.active_estops.difference_update(self.pins)
        self.active_estops.update(name for name in self.pins if words.pop(0) == 'true')
        for name in self.pins:
            is_active = name in self.active_estops
            was_active = name in previous_active_estops
            if is_active and not was_active:
                self.ESTOP_TRIGGERED.emit(name)
            elif not is_active and was_active:
                self.ESTOP_RELEASED.emit(name)


class EStopSimulation(EStop, ModuleSimulation):
    """Simulation of the e-stop module."""

    async def activate(self) -> None:
        """Activate the soft e-stop."""
        await self.set_soft_estop(True)

    async def deactivate(self) -> None:
        """Deactivate the soft e-stop."""
        await self.set_soft_estop(False)
