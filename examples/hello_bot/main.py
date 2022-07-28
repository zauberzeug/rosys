#!/usr/bin/env python3
import os

import rosys
from nicegui import ui
from rosys.automation import AutomationControls, Automator
from rosys.driving import Driver, Joystick, KeyboardControl, Odometer, RobotObject, RobotShape, Steerer
from rosys.hardware import RobotBrain, WheelsHardware, WheelsSimulation
from rosys.hardware.communication import SerialCommunication

import log_configuration

log_configuration.setup()

# setup
shape = RobotShape()
odometer = Odometer()
if SerialCommunication.is_possible():
    communication = SerialCommunication()
    robot_brain = RobotBrain(communication)
    wheels = WheelsHardware(odometer, robot_brain)
else:
    wheels = WheelsSimulation(odometer)
steerer = Steerer(wheels)
driver = Driver(wheels, odometer)
automator = Automator(wheels, steerer, default_automation=driver.drive_square)

# ui
rosys.NEW_NOTIFICATION.register(ui.notify)
KeyboardControl(steerer)
with ui.card():
    with ui.row():
        state = ui.label()
        ui.timer(0.1, lambda: state.set_text(f'{rosys.time():.3f} s, {odometer.prediction}'))

    with ui.row():
        with ui.scene():
            RobotObject(shape, odometer)
        Joystick(steerer, size=50, color='blue')

    with ui.row():
        AutomationControls(automator)
        if isinstance(wheels, WheelsHardware):
            ui.button('configure microcontroller', on_click=robot_brain.configure).props('outline')
        ui.button('restart rosys', on_click=lambda: os.utime('main.py')).props('outline')

# start
ui.on_startup(rosys.startup)
ui.on_shutdown(rosys.shutdown)
ui.run(title='hello_bot')
