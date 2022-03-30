from ..communication import Communication
from ..world import World
from .hardware import Hardware


class CommunicatingHardware(Hardware):

    def __init__(self, world: World, communication: Communication):
        super().__init__(world)
        self.communication = communication

    @property
    def is_simulation(self):
        return False
