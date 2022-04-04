from ..communication import Communication
from ..world import World
from .hardware import Hardware


class CommunicatingHardware(Hardware):

    def __init__(self, world: World, communication: Communication) -> None:
        super().__init__(world)
        self.communication = communication

    @property
    def is_simulation(self) -> bool:
        return False
