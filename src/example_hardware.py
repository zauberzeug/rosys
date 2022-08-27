#!/usr/bin/env python3
from nicegui import ui
from rosys.driving import KeyboardControl, Odometer, Steerer
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

KeyboardControl(steerer)

if is_real:
    communication.debug_ui()

    async def configure():
        await robot_brain.configure('example_hardware.txt')
    ui.button('Configure Lizard', on_click=configure).props('outline')

ui.run(title='RoSys')
