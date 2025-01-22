from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from nicegui import ui

from .. import helpers, rosys
from ..event import Event
from ..geometry import Rotation
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain

if TYPE_CHECKING:
    from .wheels import WheelsSimulation


@dataclass
class ImuMeasurement:
    """Imu measurement data with corrected and uncorrected angles and angular velocities in radians."""
    time: float
    roll: float
    pitch: float
    yaw: float
    roll_corrected: float
    pitch_corrected: float
    yaw_corrected: float
    roll_velocity: float | None
    pitch_velocity: float | None
    yaw_velocity: float | None


class Imu(Module):

    def __init__(self, offset_rotation: Rotation | None = None, **kwargs) -> None:
        super().__init__(**kwargs)
        self.offset_rotation = offset_rotation or Rotation.zero()
        self.gyro_calibration: float = 0.0
        self.last_measurement: ImuMeasurement | None = None

        self.NEW_MEASUREMENT = Event()
        """a new measurement has been received (argument: ImuMeasurement)"""

    def _emit_measurement(self, rotation: Rotation, time: float) -> None:
        assert self.offset_rotation is not None
        corrected_rotation = rotation * self.offset_rotation.T
        corrected_euler = corrected_rotation.euler
        new_measurement = ImuMeasurement(
            time=time,
            roll=rotation.roll,
            pitch=rotation.pitch,
            yaw=rotation.yaw,
            roll_corrected=corrected_euler[0],
            pitch_corrected=corrected_euler[1],
            yaw_corrected=corrected_euler[2],
            roll_velocity=None,
            pitch_velocity=None,
            yaw_velocity=None,
        )
        if self.last_measurement is not None:
            d_t = time - self.last_measurement.time
            d_roll = helpers.angle(self.last_measurement.roll, rotation.roll)
            d_pitch = helpers.angle(self.last_measurement.pitch, rotation.pitch)
            d_yaw = helpers.angle(self.last_measurement.yaw, rotation.yaw)

            roll_velocity = d_roll / d_t
            pitch_velocity = d_pitch / d_t
            yaw_velocity = d_yaw / d_t

            new_measurement.roll_velocity = roll_velocity
            new_measurement.pitch_velocity = pitch_velocity
            new_measurement.yaw_velocity = yaw_velocity
            self.NEW_MEASUREMENT.emit(new_measurement)
        self.last_measurement = new_measurement

    def developer_ui(self) -> None:
        ui.label('IMU').classes('text-center text-bold')
        with ui.column().classes('gap-y-1'):
            ui.label().bind_text_from(self, 'last_measurement',
                                      lambda m: f'Roll: {np.rad2deg(m.roll):.2f}°' if m is not None else 'Roll: N/A')
            ui.label().bind_text_from(self, 'last_measurement',
                                      lambda m: f'Pitch: {np.rad2deg(m.pitch):.2f}°' if m is not None else 'Pitch: N/A')
            ui.label().bind_text_from(self, 'last_measurement',
                                      lambda m: f'Yaw: {np.rad2deg(m.yaw):.2f}°' if m is not None else 'Yaw: N/A')


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
        self._emit_measurement(rotation, time)


class ImuSimulation(Imu, ModuleSimulation):

    def __init__(self, *, wheels: WheelsSimulation, interval: float = 0.1, roll_noise: float = 0.0, pitch_noise: float = 0.0, yaw_noise: float = 0.0, **kwargs) -> None:
        super().__init__(**kwargs)
        self.wheels = wheels
        self._roll_noise = roll_noise
        self._pitch_noise = pitch_noise
        self._yaw_noise = yaw_noise
        rosys.on_repeat(self.simulate, interval)

    def simulate(self) -> None:
        roll = np.random.normal(0, self._roll_noise)
        pitch = np.random.normal(0, self._pitch_noise)
        yaw = self.wheels.pose.yaw + np.random.normal(0, self._yaw_noise)
        self._emit_measurement(Rotation.from_euler(roll, pitch, yaw), rosys.time())

    def developer_ui(self) -> None:
        super().developer_ui()
        with ui.column().classes('gap-y-1'):
            ui.number(label='Roll Noise', format='%.3f', prefix='± ', suffix='°') \
                .bind_value(self, '_roll_noise', forward=np.deg2rad, backward=np.rad2deg).classes('w-4/5')
            ui.number(label='Pitch Noise', format='%.3f', prefix='± ', suffix='°') \
                .bind_value(self, '_pitch_noise', forward=np.deg2rad, backward=np.rad2deg).classes('w-4/5')
            ui.number(label='Yaw Noise', format='%.3f', prefix='± ', suffix='°') \
                .bind_value(self, '_yaw_noise', forward=np.deg2rad, backward=np.rad2deg).classes('w-4/5')
