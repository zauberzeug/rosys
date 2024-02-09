from typing import Optional

from ..event import Event
from ..geometry import Rotation
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain


class Imu(Module):

    def __init__(self, rotation_offset: Rotation, **kwargs) -> None:
        super().__init__(**kwargs)

        self.rotation_offset = rotation_offset
        self.rotation: Optional[Rotation] = None
        self.gyro_calibration: float = 0.0

        self.NEW_MEASUREMENT = Event()
        """a new measurement has been received (argument: rotation)"""

    @property
    def roll(self) -> Optional[float]:
        return self.rotation.roll if self.rotation else None

    @property
    def pitch(self) -> Optional[float]:
        return self.rotation.pitch if self.rotation else None

    @property
    def yaw(self) -> Optional[float]:
        return self.rotation.yaw if self.rotation else None

    def emit_measurement(self) -> None:
        self.NEW_MEASUREMENT.emit(self.rotation)


class ImuHardware(Imu, ModuleHardware):

    def __init__(self, robot_brain: RobotBrain, name: str = 'imu', **kwargs) -> None:
        self.name = name
        self.lizard_code = f'{name} = Imu()'
        self.core_message_fields = [
            f'{name}.roll',
            f'{name}.pitch',
            f'{name}.yaw',
            f'{name}.cal_gyro',
        ]
        super().__init__(robot_brain=robot_brain, lizard_code=self.lizard_code, core_message_fields=self.core_message_fields, **kwargs)

    def handle_core_output(self, time: float, words: list[str]) -> None:
        roll = float(words.pop(0))
        pitch = float(words.pop(0))
        yaw = float(words.pop(0))
        self.gyro_calibration = float(words.pop(0))
        if self.gyro_calibration < 1.0:
            return

        self.rotation = self.rotation_offset * Rotation.from_euler(roll, pitch, yaw)
        self.emit_measurement()


class ImuSimulation(Imu, ModuleSimulation):

    def set_rotation(self, rotation: Rotation) -> None:
        self.rotation = self.rotation_offset * rotation
        self.emit_measurement()
