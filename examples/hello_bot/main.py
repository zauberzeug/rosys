#!/usr/bin/env python3
from nicegui import ui
import os
import rosys
import rosys.ui
from rosys.automations import drive_square
from rosys.hardware import CommunicatingHardware
from rosys.world import Mode, World

import log_configuration
log_configuration.setup()

world = World(mode=Mode.SIMULATION)
runtime = rosys.Runtime(world)
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
            runtime.automator.replace(drive_square(world, runtime.hardware))
            await runtime.resume()

        async def configure():
            await runtime.hardware.configure()

        ui.button('drive square', on_click=play).props('icon=play_arrow')
        if isinstance(runtime.hardware, CommunicatingHardware):
            ui.button('configure microcontroller', on_click=configure).props('outline')
        ui.button('restart rosys', on_click=lambda: os.utime('main.py')).props('outline')

ui.run(title='hello_bot', port=8080)
