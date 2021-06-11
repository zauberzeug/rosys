#!/usr/bin/env python3
from nicegui import ui
import justpy as jp
import os
from rosys.runtime import Runtime
from rosys.world.mode import Mode
from rosys.ui.joystick import Joystick
from rosys.ui.three import Three

has_esp = os.path.exists('/dev/esp') and os.stat('/dev/esp').st_gid > 0
runtime = Runtime(Mode.REAL if has_esp else Mode.SIMULATION)

state = ui.label()
ui.timer(0.1, lambda: state.set_text(f'''
    {runtime.world.time:.3f} s
    (x={runtime.world.robot.pose.x:.3f},
    y={runtime.world.robot.pose.y:.3f})
'''))

Joystick(size=50, color='blue', on_drive=lambda linear, angular: jp.run_task(runtime.esp.drive(linear, angular)))

three = Three(runtime.world.robot.pose)
ui.timer(0.1, lambda: three.move_robot(runtime.world.robot.pose))


@jp.app.on_event('startup')
def startup():
    jp.run_task(runtime.run())
