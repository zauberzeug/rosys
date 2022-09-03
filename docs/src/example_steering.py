#!/usr/bin/env python3
from nicegui import ui
from rosys.driving import Odometer, Steerer, joystick, keyboard_control, robot_object
from rosys.geometry import Prism
from rosys.hardware import WheelsSimulation

shape = Prism.default_robot_shape()
wheels = WheelsSimulation()
odometer = Odometer(wheels)
steerer = Steerer(wheels)

keyboard_control(steerer)
joystick(steerer, size=50, color='blue')
with ui.scene():
    robot_object(shape, odometer)

ui.run(title='RoSys')
