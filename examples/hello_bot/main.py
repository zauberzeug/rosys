#!/usr/bin/env python3
import os
from nicegui import ui
import rosys
import rosys.ui
from rosys.automations.square import drive_square
from rosys.runtime import Runtime
from rosys.ui.joystick import Joystick
from rosys.ui.robot_object import RobotObject
from rosys.world.mode import Mode
from rosys.world.robot import Robot
from rosys.world.world import World
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
        ui.timer(0.1, lambda: state.set_text(f'''{world.time:.3f} s, {world.robot.prediction}'''))

    with ui.row():
        with ui.scene() as scene:
            robot = rosys.ui.robot_object()
            ui.timer(0.05, robot.update)
        Joystick(size=50, color='blue', steerer=runtime.steerer)

    with ui.row():
        def play(_):
            runtime.automator.replace(drive_square(world, runtime.esp))
            runtime.resume()

        ui.button('drive square', on_click=play).props('icon=play_arrow')
        ui.button('configure esp', on_click=lambda: runtime.esp.configure(world.robot.hardware)).props('outline')
        ui.button('restart rosys', on_click=lambda: os.utime('main.py')).props('outline')

ui.run(title="hello_bot", port=8080)
