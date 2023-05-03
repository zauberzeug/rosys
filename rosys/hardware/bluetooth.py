from .module import ModuleHardware
from .robot_brain import RobotBrain


class BluetoothHardware(ModuleHardware):

    def __init__(self, robot_brain: RobotBrain, *, name: str = 'robot') -> None:
        self.name = name
        lizard_code = f'bluetooth = Bluetooth("{name}")'
        super().__init__(robot_brain, lizard_code=lizard_code)
