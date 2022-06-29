from ..actors.odometer import Odometer
from ..communication import Communication
from .hardware import Hardware


class CommunicatingHardware(Hardware):

    def __init__(self, odometer: Odometer, communication: Communication) -> None:
        super().__init__(odometer)
        self.communication = communication

    @property
    def is_simulation(self) -> bool:
        return False
