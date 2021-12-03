#!/usr/bin/env python3
from nicegui import ui
import os
from rosys import Runtime, Robot, World
from rosys.automations.square import drive_square
import rosys.ui
from hardware import hardware

import log_configuration
log_configuration.setup()


world = World(robot=Robot(hardware=hardware))
runtime = Runtime(world)
rosys.ui.configure(ui, runtime)

rosys.ui.keyboard_control()
with ui.card():
    with ui.row():
        state = ui.label()
        ui.timer(0.1, lambda: state.set_text(f'{world.time:.3f} s, {world.robot.prediction}'))

    with ui.row():
        with ui.scene() as scene:
            rosys.ui.robot_object()
        rosys.ui.joystick(size=50, color='blue', steerer=runtime.steerer)

    with ui.row():
        async def play(_):
            runtime.automator.replace(drive_square(world, runtime.esp))
            await runtime.resume()

        ui.button('drive square', on_click=play).props('icon=play_arrow')
        ui.button('configure esp', on_click=lambda: runtime.esp.configure(world.robot.hardware)).props('outline')
        ui.button('restart rosys', on_click=lambda: os.utime('main.py')).props('outline')

ui.run(title="hello_bot", port=8080)
