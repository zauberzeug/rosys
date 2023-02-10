import abc
import logging

from .robot_brain import RobotBrain


class Module(abc.ABC):

    def __init__(self) -> None:
        self.log = logging.getLogger(__name__)


class ModuleHardware(Module):
    CORE_MESSAGE_FIELDS: list[str] = []

    def __init__(self, robot_brain: RobotBrain, lizard_code: str, **kwargs) -> None:
        super().__init__(**kwargs)
        self.robot_brain = robot_brain
        self.lizard_code = lizard_code
        self.message_hooks: dict[str, callable] = {}

    @abc.abstractmethod
    async def handle_core_output(self, time: float, words: list[str]) -> list[str]:
        return words


class ModuleSimulation(Module):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    @abc.abstractmethod
    async def step(self, dt: float) -> None:
        pass
