#!/usr/bin/env python3
from nicegui import ui
from rosys.driving import Joystick, KeyboardControl, Odometer, RobotObject, Steerer
from rosys.geometry import Prism
from rosys.hardware import WheelsSimulation

shape = Prism.default_robot_shape()
wheels = WheelsSimulation()
odometer = Odometer(wheels)
steerer = Steerer(wheels)

KeyboardControl(steerer)
Joystick(steerer, size=50, color='blue')
with ui.scene():
    RobotObject(shape, odometer)

ui.run(title='RoSys')
