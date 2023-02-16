
from rosys.hardware import ModuleHardware, SerialHardware
from rosys.hardware.robot_brain import RobotBrain

from .serial import SerialHardware


class ExpanderHardware(ModuleHardware):
    '''The expander module is a simple example for a representation of real or simulated robot hardware.
    '''

    def __init__(self, robot_brain: RobotBrain, *,
                 name: str = 'p0',
                 serial: SerialHardware,
                 boot: int = 25,
                 enable: int = 14) -> None:

        self.name = name
        lizard_code = f'{name} = Expander({serial.name}, {boot}, {enable})'
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)
