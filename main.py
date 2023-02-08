#!/usr/bin/env python3
from nicegui import ui

import rosys

# setup
shape = rosys.geometry.Prism.default_robot_shape()
is_real = rosys.hardware.SerialCommunication.is_possible()
if is_real:
    communication = rosys.hardware.SerialCommunication()
    robot_brain = rosys.hardware.RobotBrain(communication)
    wheels = rosys.hardware.WheelsHardware()
    robot = rosys.hardware.RobotHardware([wheels], robot_brain)
else:
    wheels = rosys.hardware.WheelsSimulation()
    robot = rosys.hardware.RobotSimulation([wheels])
odometer = rosys.driving.Odometer(wheels)
steerer = rosys.driving.Steerer(wheels)

# ui
rosys.driving.keyboard_control(steerer)
with ui.scene():
    rosys.driving.robot_object(shape, odometer)
ui.label('hold SHIFT to steer with the keyboard arrow keys')

# start
ui.run(title='RoSys')
