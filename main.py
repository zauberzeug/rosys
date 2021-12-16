#!/usr/bin/env python3
from nicegui import ui
import rosys
import rosys.ui
from rosys.world import Mode, World

# setup
runtime = rosys.Runtime(world=World(mode=Mode.SIMULATION))
rosys.ui.configure(ui, runtime)

# keyboard control
print(rosys.ui.keyboard_control.ui)
rosys.ui.keyboard_control()

# 3d scene
with ui.scene() as scene:
    robot = rosys.ui.robot_object()
ui.label('hold SHIFT to steer with the keyboard arrow keys')

# start
ui.run(title='RoSys', port=8080)
