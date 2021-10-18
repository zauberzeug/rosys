#!/usr/bin/env python3
from datetime import datetime
from nicegui import ui
from rosys import Runtime, World, Robot, Mode

world = World(mode=Mode.SIMULATION, robot=Robot())
runtime = Runtime(world)

status = ui.label()
ui.timer(0.1, lambda: status.set_text(f'''{datetime.utcfromtimestamp(world.time)} s'''))

ui.on_startup(runtime.start())
ui.on_shutdown(runtime.stop())

ui.run(title="RoSys", port=8080)
