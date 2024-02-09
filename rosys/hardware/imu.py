from .module import ModuleHardware,Module,ModuleSimulation
from .robot_brain import RobotBrain
from dataclasses import dataclass
from ..event import Event


@dataclass(slots=True, kw_only=True)
class Eulerangle:
    roll: float
    pitch: float
    yaw: float

class IMU(Module):
    def __init__(self, **kwargs) -> None:
        self.ROBOT_ANGLES = Event()
        super().__init__(**kwargs)

class ImuHardware(IMU,ModuleHardware):

    def __init__(self, robot_brain: RobotBrain,pitch_offset: float , roll_offset: float,name: str = 'imu',**kwargs) -> None:
        self.name = name
        self.pitch_offset = pitch_offset
        self.roll_offset = roll_offset
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.cal_gyro = 0
        self.lizard_code = f'{name} = Imu()'
        self.core_message_fields = [f'{name}.roll',
                                f'{name}.pitch',
                                f'{name}.yaw',
                                f'{name}.cal_gyro',
                                ]
        super().__init__(robot_brain=robot_brain,lizard_code=self.lizard_code, core_message_fields=self.core_message_fields,**kwargs)

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.roll = float(words.pop(0))
        self.pitch = float(words.pop(0))
        self.yaw = float(words.pop(0))
        self.cal_gyro = float(words.pop(0))

        self.offset_correction()
            
        angles = Eulerangle(
            roll = self.roll,
            pitch = self.pitch,
            yaw = self.yaw)
            
        if self.cal_gyro > 1.0:
            self.ROBOT_ANGLES.emit(angles)
    
    def offset_correction(self)->None:
        self.roll = self.roll + self.roll_offset
        self.pitch = self.pitch + self.pitch_offset

class IMUSimulation(IMU,ModuleSimulation):

    def __init__(self, *,
                 name: str = 'imu') -> None:
        self.name = name
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.cal_gyro = 3
        super().__init__(**kwargs)
    
    async def set_euler_angles(self, yaw : float, pitch : float, roll : float):
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        angles = Eulerangle(
            roll = self.roll,
            pitch = self.pitch,
            yaw = self.yaw)
        self.ROBOT_ANGLES.emit(angles)
    
    async def testemit(self) -> None:
        angles = Eulerangle(
            roll = self.roll,
            pitch = self.pitch,
            yaw = self.yaw)
        self.ROBOT_ANGLES.emit(angles)
