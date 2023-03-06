from .module import ModuleHardware
from .robot_brain import RobotBrain
from .serial import SerialHardware


class ExpanderHardware(ModuleHardware):
    """The expander module represents a second ESP microcontroller connected to the core ESP via serial."""

    def __init__(self, robot_brain: RobotBrain, *,
                 name: str = 'p0',
                 serial: SerialHardware,
                 boot: int = 25,
                 enable: int = 14) -> None:
        self.name = name
        lizard_code = f'{name} = Expander({serial.name}, {boot}, {enable})'
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)
