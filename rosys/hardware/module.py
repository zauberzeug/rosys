import abc
import logging
from collections.abc import Callable

from .robot_brain import RobotBrain


class Module(abc.ABC):

    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger(__name__)


class ModuleHardware(Module):

    def __init__(self, robot_brain: RobotBrain, lizard_code: str, core_message_fields: list[str] | None = None,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.robot_brain = robot_brain
        self.lizard_code = lizard_code
        self.core_message_fields: list[str] = core_message_fields or []
        self.message_hooks: dict[str, Callable] = {}

    def handle_core_output(self, time: float, words: list[str]) -> None:  # pylint: disable=unused-argument
        pass


class ModuleSimulation(Module):

    async def step(self, dt: float) -> None:  # pylint: disable=unused-argument
        pass
