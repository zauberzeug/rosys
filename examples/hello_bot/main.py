#!/usr/bin/env python3
import os

import log_configuration
from nicegui import ui

import rosys
from rosys.automation import Automator, automation_controls
from rosys.driving import Driver, Odometer, PathSegment, Steerer, joystick, keyboard_control, robot_object
from rosys.geometry import Point, Prism, Spline
from rosys.hardware import (
    CanHardware,
    RobotBrain,
    RobotHardware,
    RobotSimulation,
    SerialCommunication,
    WheelsHardware,
    WheelsSimulation,
)


async def drive_square() -> None:
    await driver.drive_path([
        PathSegment(spline=Spline.from_points(Point(x=0, y=0), Point(x=4, y=0))),
        PathSegment(spline=Spline.from_points(Point(x=4, y=0), Point(x=4, y=4))),
        PathSegment(spline=Spline.from_points(Point(x=4, y=4), Point(x=0, y=4))),
        PathSegment(spline=Spline.from_points(Point(x=0, y=4), Point(x=0, y=0))),
    ], stop_at_end=True)

log_configuration.setup()

# setup
shape = Prism.default_robot_shape()
if SerialCommunication.is_possible():
    communication = SerialCommunication()
    robot_brain = RobotBrain(communication)
    can = CanHardware(robot_brain)
    wheels = WheelsHardware(robot_brain, can=can)
    robot = RobotHardware([can, wheels], robot_brain)
else:
    wheels = WheelsSimulation()
    robot = RobotSimulation([wheels])
steerer = Steerer(wheels)
odometer = Odometer(wheels)
driver = Driver(wheels, odometer)
automator = Automator(steerer, default_automation=drive_square, on_interrupt=wheels.stop)

# ui
with ui.card():
    keyboard_control(steerer)

    with ui.row():
        state = ui.label()
        ui.timer(0.1, lambda: state.set_text(f'{rosys.time():.3f} s, {odometer.prediction}'))

    with ui.row():
        with ui.scene():
            robot_object(shape, odometer)
        joystick(steerer, size=50, color='blue')

    with ui.row():
        automation_controls(automator)
        if isinstance(wheels, WheelsHardware):
            ui.button('configure microcontroller', on_click=robot_brain.configure).props('outline')
        ui.button('restart rosys', on_click=lambda: os.utime('main.py')).props('outline')

# start
ui.run(title='hello_bot')
