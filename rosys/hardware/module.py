import abc
import logging

from .robot_brain import RobotBrain


class Module(abc.ABC):

    def __init__(self) -> None:
        self.log = logging.getLogger(__name__)


class ModuleHardware(Module):

    def __init__(self, robot_brain: RobotBrain, **kwargs) -> None:
        super().__init__(**kwargs)
        self.robot_brain = robot_brain
        self.serial_hooks: dict[str, callable] = {}

    @abc.abstractmethod
    async def handle_core_output(self, time: float, words: list[str]) -> None:
        pass


class ModuleSimulation(Module):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    @abc.abstractmethod
    async def step(self, dt: float) -> None:
        pass
