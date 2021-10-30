#!/usr/bin/env python3
from nicegui import ui
import rosys
import rosys.ui

world = rosys.World(robot=rosys.Robot())
runtime = rosys.Runtime(world)

status = ui.label()
ui.timer(0.1, lambda: status.set_text(f'''{world.robot.prediction}'''))

rosys.ui.joystick(size=50, color='blue', steerer=runtime.steerer)
with ui.scene() as scene:
    robot = rosys.ui.robot_object(world.robot)
    ui.timer(0.05, robot.update)

rosys.ui.keyboard_control(ui, steerer=runtime.steerer)

ui.on_startup(runtime.start())
ui.on_shutdown(runtime.stop())

ui.run(title="RoSys", port=8080)
