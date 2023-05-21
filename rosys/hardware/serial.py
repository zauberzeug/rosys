from .module import ModuleHardware
from .robot_brain import RobotBrain


class SerialHardware(ModuleHardware):
    """The serial module represents a serial connection with another device."""

    def __init__(self, robot_brain: RobotBrain, *,
                 name: str = 'serial',
                 rx_pin: int = 26,
                 tx_pin: int = 27,
                 baud: int = 115_200,
                 num: int = 1) -> None:

        self.name = name
        lizard_code = f'{name} = Serial({rx_pin}, {tx_pin}, {baud}, {num})'
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)
