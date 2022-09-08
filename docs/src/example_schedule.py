#!/usr/bin/env python3
from datetime import datetime

from nicegui import ui
from rosys.automation import Automator, Schedule, automation_controls
from rosys.driving import Driver, Odometer, robot_object
from rosys.geometry import Point, Prism
from rosys.hardware import WheelsSimulation


async def forward_backward() -> None:
    while True:
        await driver.drive_to(Point(x=1, y=0))
        await driver.drive_to(Point(x=0, y=0))


async def stop() -> None:
    await wheels.drive(0, 0)

shape = Prism.default_robot_shape()
wheels = WheelsSimulation()
odometer = Odometer(wheels)
driver = Driver(wheels, odometer)
automator = Automator(wheels, None, default_automation=forward_backward)

schedule = Schedule(automator, on_enable=forward_backward, on_disable=stop)
schedule.plan.disable_all()

ui.label(
    f'You can test the schedule by clicking on slot "{datetime.now().hour:02d}" in row {datetime.now().weekday()+1}. '
    f'This will let the robot drive back and forth for the current hour.')
schedule.ui()
with ui.scene():
    robot_object(shape, odometer)
with ui.row():
    automation_controls(automator)

ui.run(title='RoSys')
