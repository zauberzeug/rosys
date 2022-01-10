from typing import Optional
from ..world import World
from ..communication import Communication
from .hardware import Hardware


class CommunicatingHardware(Hardware):

    def __init__(self, world: World, communication: Optional[Communication]):
        super().__init__(world)
        self.communication = communication
