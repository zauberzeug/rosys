#!/usr/bin/env python3
import math

from nicegui import ui

import rosys
import rosys.helpers


async def run() -> None:
    while True:
        await forward()
        await turn_left()


async def forward() -> None:
    start = odometer.prediction
    while start.distance(odometer.prediction) < 1.0:
        await wheels.drive(1.0, 0.0)
        await rosys.sleep(0.1)
    await wheels.stop()


@rosys.automation.uninterruptible
async def turn_left() -> None:
    start = odometer.prediction
    while rosys.helpers.angle(start.yaw, odometer.prediction.yaw) < math.radians(90):
        await wheels.drive(0.0, 1.0)
        await rosys.sleep(0.1)
    await wheels.stop()


shape = rosys.geometry.Prism.default_robot_shape()
wheels = rosys.hardware.WheelsSimulation()
robot = rosys.hardware.RobotSimulation([wheels])
odometer = rosys.driving.Odometer(wheels)
driver = rosys.driving.Driver(wheels, odometer)
automator = rosys.automation.Automator(None, default_automation=run, on_interrupt=wheels.stop)

with ui.scene():
    rosys.driving.robot_object(shape, odometer)
with ui.row():
    rosys.automation.automation_controls(automator)

ui.run(title='Automations')
