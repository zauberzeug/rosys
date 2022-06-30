#!/usr/bin/env python3
import os

import rosys
import rosys.ui
from nicegui import ui
from rosys.actors import Automator, Driver, Odometer, Steerer
from rosys.hardware import RobotBrain, WheelsHardware, WheelsSimulation
from rosys.hardware.communication import SerialCommunication

import log_configuration

log_configuration.setup()

# setup
robot = rosys.world.Robot()
odometer = Odometer()
if SerialCommunication.is_possible():
    communication = SerialCommunication()
    robot_brain = RobotBrain(communication)
    wheels = WheelsHardware(odometer, robot_brain)
else:
    wheels = WheelsSimulation(odometer)
steerer = Steerer(wheels)
driver = Driver(wheels)
automator = Automator()

# keyboard control
rosys.ui.keyboard_control(steerer)

# 3d scene, joystick and automation controls
with ui.card():
    with ui.row():
        state = ui.label()
        ui.timer(0.1, lambda: state.set_text(f'{rosys.time():.3f} s, {odometer.prediction}'))

    with ui.row():
        with ui.scene():
            rosys.ui.robot_object(robot, odometer)
        rosys.ui.joystick(steerer, size=50, color='blue')

    with ui.row():
        rosys.ui.automation_controls(automator, default_automation=driver.drive_square)
        if isinstance(wheels, WheelsHardware):
            ui.button('configure microcontroller', on_click=robot_brain.configure).props('outline')
        ui.button('restart rosys', on_click=lambda: os.utime('main.py')).props('outline')

ui.on_startup(rosys.startup())
ui.on_shutdown(rosys.shutdown())

ui.run(title='hello_bot')
