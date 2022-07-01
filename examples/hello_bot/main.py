#!/usr/bin/env python3
import os

import rosys
import rosys.ui
from nicegui import ui
from rosys import runtime
from rosys.actors import Automator, Driver, Odometer, Steerer
from rosys.hardware import RobotBrain, WheelsHardware, WheelsSimulation
from rosys.hardware.communication import SerialCommunication

import log_configuration

log_configuration.setup()

# actors
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

# ui
runtime.NEW_NOTIFICATION.register(ui.notify)
rosys.ui.keyboard_control(steerer)
with ui.card():
    with ui.row():
        state = ui.label()
        ui.timer(0.1, lambda: state.set_text(f'{runtime.time:.3f} s, {odometer.prediction}'))

    with ui.row():
        with ui.scene():
            rosys.ui.robot_object(robot, odometer)
        rosys.ui.joystick(steerer, size=50, color='blue')

    with ui.row():
        rosys.ui.automation_controls(automator, default_automation=driver.drive_square)
        if isinstance(wheels, WheelsHardware):
            ui.button('configure microcontroller', on_click=robot_brain.configure).props('outline')
        ui.button('restart rosys', on_click=lambda: os.utime('main.py')).props('outline')

# start
ui.on_startup(runtime.startup())
ui.on_shutdown(runtime.shutdown())
ui.run(title='hello_bot')
