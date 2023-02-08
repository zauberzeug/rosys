#!/usr/bin/env python3
from nicegui import ui
from wheels_for_custom_hardware import WheelsForCustomHardware

from rosys.driving import Odometer, Steerer, joystick, keyboard_control
from rosys.hardware import WheelsSimulation

try:
    wheels = WheelsForCustomHardware()
except Exception:
    wheels = WheelsSimulation()
odometer = Odometer(wheels)
steerer = Steerer(wheels)

keyboard_control(steerer)
joystick(steerer)

ui.run(title='RoSys')
