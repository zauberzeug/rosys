import abc
import logging

from .robot_brain import RobotBrain


class Module(abc.ABC):

    def __init__(self) -> None:
        self.log = logging.getLogger(__name__)


class ModuleHardware(Module):

    def __init__(
            self, robot_brain: RobotBrain, lizard_code: str, core_message_fields: list[str] = [],
            **kwargs) -> None:
        super().__init__(**kwargs)
        self.robot_brain = robot_brain
        self.lizard_code = lizard_code
        self.core_message_fields: list[str] = core_message_fields
        self.message_hooks: dict[str, callable] = {}

    def handle_core_output(self, time: float, words: list[str]) -> None:
        pass


class ModuleSimulation(Module):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    async def step(self, dt: float) -> None:
        pass
