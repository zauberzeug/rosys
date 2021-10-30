#!/usr/bin/env python3
from nicegui import ui
import rosys
import rosys.ui

world = rosys.World(robot=rosys.Robot())
runtime = rosys.Runtime(world)
rosys.ui.configure(ui, runtime)
rosys.ui.keyboard_control()

with ui.scene() as scene:
    robot = rosys.ui.robot_object(world.robot)
    ui.timer(0.05, robot.update)
ui.label('hold SHIFT to steer with the keyboard arrow keys')

ui.run(title="RoSys", port=8080)
