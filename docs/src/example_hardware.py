#!/usr/bin/env python3
from nicegui import ui
from rosys.driving import Odometer, Steerer, joystick, keyboard_control
from rosys.hardware import WheelsSimulation

from wheels_for_custom_hardware import WheelsForCustomHardware

try:
    wheels = WheelsForCustomHardware()
except:
    wheels = WheelsSimulation()
odometer = Odometer(wheels)
steerer = Steerer(wheels)

keyboard_control(steerer)
joystick(steerer)

ui.run(title='RoSys')
