from ..world import World
from ..communication import Communication
from .hardware import Hardware


class CommunicatingHardware(Hardware):

    def __init__(self, world: World, communication: Communication):
        super().__init__(world)
        self.communication = communication
