#!/usr/bin/env python3
import os

import rosys
from nicegui import ui
from rosys.automation import Automator, automation_controls
from rosys.driving import Driver, Odometer, Steerer, joystick, keyboard_control, robot_object
from rosys.geometry import Prism
from rosys.hardware import RobotBrain, SerialCommunication, WheelsHardware, WheelsSimulation

import log_configuration

log_configuration.setup()

# setup
shape = Prism.default_robot_shape()
if SerialCommunication.is_possible():
    communication = SerialCommunication()
    robot_brain = RobotBrain(communication)
    wheels = WheelsHardware(robot_brain)
else:
    wheels = WheelsSimulation()
steerer = Steerer(wheels)
odometer = Odometer(wheels)
driver = Driver(wheels, odometer)
automator = Automator(wheels, steerer, default_automation=driver.drive_square)

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
