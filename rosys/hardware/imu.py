from ..event import Event
from ..geometry import Rotation
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain


class Imu(Module):

    def __init__(self, offset_rotation: Rotation, **kwargs) -> None:
        super().__init__(**kwargs)
        self.offset_rotation = offset_rotation
        self.rotation: Rotation | None = None
        self.gyro_calibration: float = 0.0

        self.NEW_MEASUREMENT = Event()
        """a new measurement has been received (argument: Tuple[Roll, Pitch, Yaw])"""

    @property
    def roll(self) -> float | None:
        return self.rotation.roll if self.rotation else None

    @property
    def pitch(self) -> float | None:
        return self.rotation.pitch if self.rotation else None

    @property
    def yaw(self) -> float | None:
        return self.rotation.yaw if self.rotation else None

    @property
    def euler(self) -> tuple[float, float, float] | None:
        return self.rotation.euler if self.rotation else None

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
        rotation = Rotation.from_quaternion(
            float(words.pop(0)),
            float(words.pop(0)),
            float(words.pop(0)),
            float(words.pop(0)),
        )

        if self.gyro_calibration < 1.0:
            return

        self.rotation = rotation * self.offset_rotation.T
        self.emit_measurement()


class ImuSimulation(Imu, ModuleSimulation):

    def simulate_measurement(self, rotation: Rotation) -> None:
        self.rotation = rotation * self.offset_rotation.T
        self.emit_measurement()
