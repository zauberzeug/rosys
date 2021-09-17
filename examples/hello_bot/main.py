#!/usr/bin/env python3
import os
from nicegui import ui
from rosys.runtime import Runtime
from rosys.ui.joystick import Joystick
from rosys.world.mode import Mode
from rosys.world.robot import Robot
from rosys.world.world import World
from rosys.automations.square import drive_square
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
        with ui.scene() as scene:
            outline = list(map(list, world.robot.shape.outline))
            robot = scene.extrusion(outline, world.robot.shape.height).material('#4488ff')
            ui.timer(0.05, lambda: robot
                     .move(world.robot.prediction.x, world.robot.prediction.y)
                     .rotate(0, 0, world.robot.prediction.yaw) and False)
        Joystick(size=50, color='blue', steerer=runtime.steerer)

    ui.button('configure esp', on_click=lambda: runtime.esp.configure(world.robot.hardware))
    ui.button('restart rosys', on_click=lambda: os.utime('main.py'))

    def play(_):
        runtime.automator.add(drive_square(world, runtime.esp))
        runtime.resume()

    ui.button('drive square', on_click=play)


ui.on_startup(runtime.run())
ui.on_shutdown(runtime.stop())

ui.run(title="hello_bot")
