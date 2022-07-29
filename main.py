#!/usr/bin/env python3
from nicegui import ui

from rosys.driving import KeyboardControl, Odometer, RobotObject, RobotShape, Steerer
from rosys.hardware import RobotBrain, SerialCommunication, WheelsHardware, WheelsSimulation, communication

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

# ui
KeyboardControl(steerer)
with ui.scene():
    RobotObject(shape, odometer)
ui.label('hold SHIFT to steer with the keyboard arrow keys')

# start
ui.run(title='RoSys')
