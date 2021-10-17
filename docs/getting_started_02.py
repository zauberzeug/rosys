#!/usr/bin/env python3
from datetime import datetime
from nicegui import ui
from rosys import Runtime, World, Robot, Mode
from rosys.ui import Joystick, RobotObject

world = World(mode=Mode.SIMULATION, robot=Robot())
runtime = Runtime(world)

status = ui.label()
ui.timer(0.1, lambda: status.set_text(f'''{datetime.utcfromtimestamp(world.time)} s'''))

Joystick(size=50, color='blue', steerer=runtime.steerer)
with ui.scene() as scene:
    robot = RobotObject(world.robot)
    ui.timer(0.05, robot.update)

ui.on_startup(runtime.run())
ui.on_shutdown(runtime.stop())

ui.run(title="RoSys", port=8080)
