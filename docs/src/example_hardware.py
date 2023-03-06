#!/usr/bin/env python3
from nicegui import ui

import rosys


class CustomWheelsHardware(rosys.hardware.Wheels):

    def __init__(self) -> None:
        super().__init__()
        rosys.on_repeat(self.read_current_velocity, 0.01)

    async def drive(self, linear: float, angular: float) -> None:
        await super().drive(linear, angular)
        # TODO send hardware command to drive with given linear and angular velocity

    async def stop(self) -> None:
        await super().stop()
        # TODO send hardware command to stop the wheels

    async def read_current_velocity(self) -> None:
        velocities: list[rosys.geometry.Velocity] = []
        # TODO: read measured velocities from the hardware
        self.VELOCITY_MEASURED.emit(velocities)


try:
    wheels = CustomWheelsHardware()
    robot = rosys.hardware.Robot([wheels])
except Exception:
    wheels = rosys.hardware.WheelsSimulation()
    robot = rosys.hardware.RobotSimulation([wheels])
odometer = rosys.driving.Odometer(wheels)
steerer = rosys.driving.Steerer(wheels)

rosys.driving.keyboard_control(steerer)
rosys.driving.joystick(steerer)
ui.label().bind_text_from(wheels, 'linear_target_speed', lambda l: f'Linear: {l:.2f} m/s')
ui.label().bind_text_from(wheels, 'angular_target_speed', lambda a: f'Angular: {a:.2f} rad/s')

ui.run(title='RoSys')
