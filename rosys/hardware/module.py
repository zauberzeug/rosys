import abc
import logging


class Module(abc.ABC):

    def __init__(self) -> None:
        self.log = logging.getLogger(__name__)


class ModuleHardware(Module):

    @abc.abstractmethod
    async def handle_core_output(self, time: float, words: list[str]) -> None:
        pass


class ModuleSimulation(Module):

    @abc.abstractmethod
    async def step(self, dt: float) -> None:
        pass
