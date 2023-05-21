#!/usr/bin/env python3
from nicegui import ui

import rosys

# setup
shape = rosys.geometry.Prism.default_robot_shape()
rosys.hardware.SerialCommunication.search_paths = ['/dev/ttyUSB0']
is_real = rosys.hardware.SerialCommunication.is_possible()
if is_real:
    communication = rosys.hardware.SerialCommunication()
    robot_brain = rosys.hardware.RobotBrain(communication)
    can = rosys.hardware.CanHardware(robot_brain)
    wheels = rosys.hardware.WheelsHardware(robot_brain,
                                           can=can,
                                           left_can_address=0x100,
                                           right_can_address=0x000,
                                           m_per_tick=0.01571,
                                           width=0.207,
                                           is_right_reversed=True)
    robot = rosys.hardware.RobotHardware([can, wheels], robot_brain)
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
if is_real:
    ui.button('configure microcontroller', on_click=robot.configure).props('outline')

# start
ui.run(title='RoSys')
