#!/usr/bin/env python3
import rosys.ui
from nicegui import ui
from rosys import runtime
from rosys.actors import Odometer
from rosys.world import Robot, RobotShape

# setup
shape = RobotShape(outline=[
    (0, 0), (-0.5, -0.5), (1.5, -0.5), (1.75, 0), (1.5, 0.5), (-0.5, 0.5),
])  # the shape for the robot will be used in 3d rendering
robot = Robot(shape=shape)
odometer = Odometer()

# ui
with ui.scene():
    # `debug=True` turns on a wireframe, the axes-center and follow-the-line target
    rosys.ui.robot_object(robot, odometer, debug=True)

# start
ui.on_startup(runtime.startup())
ui.on_shutdown(runtime.shutdown())
ui.run(port=8080)
