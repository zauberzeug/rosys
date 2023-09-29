from .module import ModuleHardware
from .robot_brain import RobotBrain


class ImuHardware(ModuleHardware):

    def __init__(self, robot_brain: RobotBrain) -> None:
        lizard_code = 'imu = Imu()'
        super().__init__(robot_brain, lizard_code=lizard_code)
