#!/usr/bin/env python3
from nicegui import ui
import os
from rosys.runtime import Runtime
from rosys.world.mode import Mode
from rosys.ui.joystick import Joystick
from rosys.ui.three import Three
import icecream
icecream.install()

has_esp = os.path.exists('/dev/esp') and os.stat('/dev/esp').st_gid > 0
runtime = Runtime(Mode.REAL if has_esp else Mode.SIMULATION)

state = ui.label()
ui.timer(0.1, lambda: state.set_text(f'''
    {runtime.world.time:.3f} s
    (x={runtime.world.robot.pose.x:.3f},
    y={runtime.world.robot.pose.y:.3f})
'''))

cams = ui.label()
ui.timer(1, lambda: cams.set_text(f'cams: {runtime.world.cameras}'))

detections = ui.label()
ui.timer(1, lambda: detections.set_text(f'images: {runtime.world.images}'))

Joystick(size=50, color='blue', steerer=runtime.steerer)

three = Three(runtime.world.robot.pose)
ui.timer(0.05, lambda: three.set_robot_pose(runtime.world.robot.pose))

ui.on_startup(runtime.run())
ui.on_shutdown(runtime.stop())
