from typing import Optional
from pyquaternion import Quaternion
import numpy as np
from ..event import Event
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain


class Imu(Module):

    def __init__(self, offset_quaternion: Quaternion, **kwargs) -> None:
        super().__init__(**kwargs)
        self.offset_quaternion = offset_quaternion
        self.positional_quaternion: Optional[Quaternion] = None
        self.gyro_calibration: float = 0.0

        self.NEW_MEASUREMENT = Event()
        """a new measurement has been received (argument: rotation)"""

    @property
    def roll(self) -> Optional[float]:
        if not self.positional_quaternion:
            return None
        else:
            e = self.positional_quaternion.elements
            t0 = +2.0 * (e[0] * e[1] + e[2] * e[3])
            t1 = +1.0 - 2.0 * (e[1] * e[1] + e[2] * e[2])
            return np.degrees(np.arctan2(t0, t1))

    @property
    def pitch(self) -> Optional[float]:
        if not self.positional_quaternion:
            return None
        else:
            e = self.positional_quaternion.elements
            t2 = +2.0 * (e[0] * e[2] - e[3] * e[1])
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            return np.degrees(np.arcsin(t2))

    @property
    def yaw(self) -> Optional[float]:
        if not self.positional_quaternion:
            return None
        else:
            e = self.positional_quaternion.elements
            t3 = +2.0 * (e[0] * e[3] + e[1] * e[2])
            t4 = +1.0 - 2.0 * (e[2] * e[2] + e[3] * e[3])
            return np.degrees(np.arctan2(t3, t4))

    @property
    def euler(self) -> Optional[tuple[float, float, float]]:
        if not self.positional_quaternion:
            return None
        else:
            return self.roll, self.pitch, self.yaw

    def emit_measurement(self) -> None:
        self.NEW_MEASUREMENT.emit(self.euler)


class ImuHardware(Imu, ModuleHardware):

    def __init__(self, robot_brain: RobotBrain, name: str = 'imu', **kwargs) -> None:
        self.name = name
        self.lizard_code = f'{name} = Imu()'
        self.core_message_fields = [
            f'{name}.cal_gyr',
            f'{name}.quat_w:4',
            f'{name}.quat_x:4',
            f'{name}.quat_y:4',
            f'{name}.quat_z:4',
        ]
        super().__init__(robot_brain=robot_brain, lizard_code=self.lizard_code, core_message_fields=self.core_message_fields, **kwargs)

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.gyro_calibration = float(words.pop(0))
        self.positional_quaternion = Quaternion((words.pop(0), words.pop(0), words.pop(0), words.pop(0)))

        if self.gyro_calibration < 1.0:
            return

        self.positional_quaternion = self.positional_quaternion * self.offset_quaternion.inverse
        self.emit_measurement()


class ImuSimulation(Imu, ModuleSimulation):

    def set_quaternion(self, quaternion) -> None:
        self.positional_quaternion = quaternion * self.offset_quaternion
        self.emit_measurement()
