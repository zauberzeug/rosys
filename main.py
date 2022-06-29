#!/usr/bin/env python3
from nicegui import ui

import rosys
import rosys.ui
from rosys.actors import Lizard, Odometer, Steerer
from rosys.communication import SerialCommunication
from rosys.hardware import RobotBrain, SimulatedHardware

# setup
robot = rosys.world.Robot()
odometer = Odometer()
hardware = \
    RobotBrain(odometer, SerialCommunication()) if SerialCommunication.is_possible() else SimulatedHardware(odometer)
lizard = Lizard(hardware)
steerer = Steerer(hardware)

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
