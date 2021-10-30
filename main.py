#!/usr/bin/env python3
from nicegui import ui
import rosys
import rosys.ui

# setup
runtime = rosys.Runtime()
rosys.ui.configure(ui, runtime)

# keyboard control
rosys.ui.keyboard_control()

# 3d scene
with ui.scene() as scene:
    robot = rosys.ui.robot_object()
    ui.timer(0.05, robot.update)
ui.label('hold SHIFT to steer with the keyboard arrow keys')

# start
ui.run(title="RoSys", port=8080)
