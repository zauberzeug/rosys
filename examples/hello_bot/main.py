#!/usr/bin/env python3
import os
from nicegui import ui
from rosys.runtime import Runtime
from rosys.ui.joystick import Joystick
from rosys.ui.three import Three
from rosys.world.mode import Mode
from rosys.world.robot import Robot
from rosys.world.world import World
from hardware import hardware

mode = Mode.REAL if os.path.exists('/dev/esp') and os.stat('/dev/esp').st_gid > 0 else Mode.SIMULATION
world = World(mode=mode, robot=Robot(hardware=hardware))

runtime = Runtime(world)

with ui.card():
    state = ui.label()
    ui.timer(0.1, lambda: state.set_text(f'''
        {world.time:.3f} s
        (x={world.robot.prediction.x:.3f},
        y={world.robot.prediction.y:.3f})
    '''))

    with ui.row():
        def update_three():
            need_updates = [
                three.set_robot('prediction', world.robot.prediction, world.robot.shape),
                three.update_carrot(world.carrot)
            ]
            return not all(n == False for n in need_updates)
        three = Three(width=640, height=480)
        ui.timer(0.05, update_three)

        Joystick(size=50, color='blue', steerer=runtime.steerer)

    ui.button('configure esp', on_click=lambda: runtime.esp.configure(world.robot.hardware))
    ui.button('restart rosys', on_click=lambda: os.utime('main.py'))

ui.on_startup(runtime.run())
ui.on_shutdown(runtime.stop())

ui.run(title="hello_bot")
