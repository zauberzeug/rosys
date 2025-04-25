from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from nicegui import ui

from .. import rosys
from ..driving.driver import PoseProvider
from ..event import Event
from ..geometry import Rotation
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain


@dataclass(slots=True, kw_only=True)
class ImuMeasurement:
    """Imu measurement data with corrected and uncorrected angles and angular velocities in radians."""
    time: float
    gyro_calibration: float
    rotation: Rotation
    angular_velocity: Rotation


class Imu(Module):
    """A module that provides measurements from an IMU."""

    def __init__(self, offset_rotation: Rotation | None = None, **kwargs) -> None:
        super().__init__(**kwargs)
        self.offset_rotation = offset_rotation or Rotation.zero()
        self.last_measurement: ImuMeasurement | None = None

        self.NEW_MEASUREMENT = Event[ImuMeasurement]()
        """a new measurement has been received (argument: ImuMeasurement)"""

    def _emit_measurement(self, gyro_calibration: float, raw_rotation: Rotation, time: float) -> None:
        new_measurement = ImuMeasurement(
            time=time,
            gyro_calibration=gyro_calibration,
            rotation=raw_rotation * self.offset_rotation.T,
            angular_velocity=Rotation.zero(),
        )
        if self.last_measurement is not None:
            d_rotation = new_measurement.rotation * self.last_measurement.rotation.T
            d_roll, d_pitch, d_yaw = d_rotation.euler
            d_t = time - self.last_measurement.time
            new_measurement.angular_velocity = Rotation.from_euler(d_roll / d_t, d_pitch / d_t, d_yaw / d_t)
        self.last_measurement = new_measurement
        self.NEW_MEASUREMENT.emit(new_measurement)

    def developer_ui(self) -> None:
        ui.label('IMU').classes('text-center text-bold')
        with ui.row().classes('gap-0 w-56'):
            with ui.column().classes('gap-y-0 w-1/3'):
                ui.label('Roll:')
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda m: f'{np.rad2deg(m.rotation.roll):.2f}°' if m is not None else 'N/A')
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda m: f'{np.rad2deg(m.angular_velocity.roll):.2f}°/s' if m is not None and m.angular_velocity.roll is not None else 'N/A')
            with ui.column().classes('gap-y-0 w-1/3'):
                ui.label('Pitch:')
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda m: f'{np.rad2deg(m.rotation.pitch):.2f}°' if m is not None else 'N/A')
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda m: f'{np.rad2deg(m.angular_velocity.pitch):.2f}°/s' if m is not None and m.angular_velocity.pitch is not None else 'N/A')
            with ui.column().classes('gap-y-0 w-1/3'):
                ui.label('Yaw:')
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda m: f'{np.rad2deg(m.rotation.yaw):.2f}°' if m is not None else 'N/A')
                ui.label().bind_text_from(self, 'last_measurement',
                                          lambda m: f'{np.rad2deg(m.angular_velocity.yaw):.2f}°/s' if m is not None and m.angular_velocity.yaw is not None else 'N/A')
        ui.label().bind_text_from(self, 'last_measurement',
                                  lambda m: f'Gyro-Calibration: {m.gyro_calibration:.0f}' if m is not None else 'N/A')


class ImuHardware(Imu, ModuleHardware):
    """A hardware module that handles the communication with an IMU."""

    def __init__(self, robot_brain: RobotBrain, name: str = 'imu', *, min_gyro_calibration: float = 1.0, **kwargs) -> None:
        self.name = name
        self.min_gyro_calibration = min_gyro_calibration
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
        gyro_calibration = float(words.pop(0))
        raw_rotation = Rotation.from_quaternion(
            float(words.pop(0)),
            float(words.pop(0)),
            float(words.pop(0)),
            float(words.pop(0)),
        )
        if gyro_calibration < self.min_gyro_calibration:
            return
        self._emit_measurement(gyro_calibration, raw_rotation, time)


class ImuSimulation(Imu, ModuleSimulation):
    """Simulation of an IMU."""

    def __init__(self, *,
                 wheels: PoseProvider,
                 interval: float = 0.1,
                 roll_noise: float = 0.0,
                 pitch_noise: float = 0.0,
                 yaw_noise: float = 0.0,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.wheels = wheels
        self._roll_noise = roll_noise
        self._pitch_noise = pitch_noise
        self._yaw_noise = yaw_noise
        rosys.on_repeat(self.simulate, interval)

    def simulate(self) -> None:
        roll = np.random.normal(0, self._roll_noise)
        pitch = np.random.normal(0, self._pitch_noise)
        yaw = np.random.normal(self.wheels.pose.yaw, self._yaw_noise)
        self._emit_measurement(3.0, Rotation.from_euler(roll, pitch, yaw), rosys.time())

    def developer_ui(self) -> None:
        super().developer_ui()
        with ui.column().classes('gap-y-0'):
            ui.number(label='Roll Noise', format='%.3f', prefix='± ', suffix='°') \
                .bind_value(self, '_roll_noise', forward=np.deg2rad, backward=np.rad2deg).classes('w-4/5')
            ui.number(label='Pitch Noise', format='%.3f', prefix='± ', suffix='°') \
                .bind_value(self, '_pitch_noise', forward=np.deg2rad, backward=np.rad2deg).classes('w-4/5')
            ui.number(label='Yaw Noise', format='%.3f', prefix='± ', suffix='°') \
                .bind_value(self, '_yaw_noise', forward=np.deg2rad, backward=np.rad2deg).classes('w-4/5')
