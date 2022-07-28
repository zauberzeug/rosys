#!/usr/bin/env python3
from nicegui import ui

import rosys
import rosys.ui
from rosys.actors import Odometer, Steerer
from rosys.hardware import RobotBrain, WheelsHardware, WheelsSimulation, robot_brain
from rosys.hardware.communication import SerialCommunication, communication
from rosys.world import RobotShape

# setup
shape = RobotShape()
odometer = Odometer()
if SerialCommunication.is_possible():
    communication = SerialCommunication()
    robot_brain = RobotBrain(communication)
    wheels = WheelsHardware(odometer, robot_brain)
else:
    wheels = WheelsSimulation(odometer)
steerer = Steerer(wheels)

# ui
rosys.NEW_NOTIFICATION.register(ui.notify)
rosys.ui.keyboard_control(steerer)
with ui.scene():
    rosys.ui.robot_object(shape, odometer)
ui.label('hold SHIFT to steer with the keyboard arrow keys')

# start
ui.on_startup(rosys.startup)
ui.on_shutdown(rosys.shutdown)
ui.run(title='RoSys')
