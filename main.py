#!/usr/bin/env python3
from nicegui import ui

from rosys.driving import Odometer, Steerer, keyboard_control, robot_object
from rosys.geometry import Prism
from rosys.hardware import WheelsSimulation

# setup
shape = Prism.default_robot_shape()
wheels = WheelsSimulation()
odometer = Odometer(wheels)
steerer = Steerer(wheels)

# ui
keyboard_control(steerer)
with ui.scene():
    robot_object(shape, odometer)
ui.label('hold SHIFT to steer with the keyboard arrow keys')

# start
ui.run(title='RoSys')
