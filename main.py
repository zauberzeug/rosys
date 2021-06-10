#!/usr/bin/env python3
from nicegui import ui
import justpy as jp
import os
from rosys.runtime import Runtime
from rosys.world.mode import Mode

has_esp = os.path.exists('/dev/esp') and os.stat('/dev/esp').st_gid > 0
runtime = Runtime(Mode.REAL if has_esp else Mode.SIMULATION)

state = ui.label()
ui.timer(0.1, lambda: state.set_text(f'''
    {runtime.world.time:.3f} s
    (x={runtime.world.robot.pose.x:.3f},
    y={runtime.world.robot.pose.y:.3f})
'''))

ui.slider(min=-1, max=1, step=0.1, value=0,
          on_change=lambda e: jp.run_task(runtime.esp.power(e.value, e.value)))


@jp.app.on_event('startup')
def startup():
    jp.run_task(runtime.run())
