#!/usr/bin/env python3
from nicegui import ui

import rosys
import rosys.ui
from rosys.actors import Odometer, Steerer
from rosys.hardware import RobotBrain, WheelsHardware, WheelSimulation, robot_brain
from rosys.hardware.communication import SerialCommunication, communication

# setup
robot = rosys.world.Robot()
odometer = Odometer()
if SerialCommunication.is_possible():
    communication = SerialCommunication()
    robot_brain = RobotBrain(communication)
    wheels = WheelsHardware(odometer, robot_brain)
else:
    wheels = WheelSimulation(odometer)
steerer = Steerer(wheels)

# keyboard control
rosys.ui.keyboard_control(steerer)

# 3d scene
with ui.scene():
    rosys.ui.robot_object(robot, odometer)
ui.label('hold SHIFT to steer with the keyboard arrow keys')

ui.on_startup(rosys.startup())
ui.on_shutdown(rosys.shutdown())

# start
ui.run(title='RoSys')
