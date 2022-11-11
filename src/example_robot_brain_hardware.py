#!/usr/bin/env python3
from nicegui import ui

from rosys.driving import Odometer, Steerer, joystick, keyboard_control
from rosys.hardware import RobotBrain, SerialCommunication, WheelsHardware, WheelsSimulation, communication

is_real = SerialCommunication.is_possible()
if is_real:
    communication = SerialCommunication()
    robot_brain = RobotBrain(communication)
    wheels = WheelsHardware(robot_brain)
else:
    wheels = WheelsSimulation()
odometer = Odometer(wheels)
steerer = Steerer(wheels)

keyboard_control(steerer)
joystick(steerer)

if is_real:
    communication.debug_ui()
    robot_brain.developer_ui()

ui.run(title='RoSys')
