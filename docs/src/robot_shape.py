#!/usr/bin/env python3
from nicegui import ui
from rosys.driving import Odometer, RobotObject
from rosys.geometry import Prism
from rosys.hardware import WheelsSimulation

# setup
shape = Prism(outline=[(0, 0), (-0.5, -0.5), (1.5, -0.5), (1.75, 0), (1.5, 0.5), (-0.5, 0.5)],
              height=0.5)  # the shape for the robot will be used in 3d rendering
wheels = WheelsSimulation()
odometer = Odometer(wheels)

# ui
with ui.scene():
    # `debug=True` turns on a wireframe, the axes-center and follow-the-line target
    RobotObject(shape, odometer, debug=True)

# start
ui.run()
