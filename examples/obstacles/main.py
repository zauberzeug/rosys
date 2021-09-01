#!/usr/bin/env python3
import asyncio
import os
import starlette
from nicegui import app, ui
from rosys.automations.drive_path import drive_to
from rosys.runtime import Runtime
from rosys.ui.joystick import Joystick
from rosys.world.mode import Mode
from rosys.world.point import Point
from rosys.world.robot import Robot
from rosys.world.world import World, WorldState

import icecream
icecream.install()

world = World(mode=Mode.SIMULATION, robot=Robot())

runtime = Runtime(world, backup_filepath='/tmp/world.json')

with ui.card():
    state = ui.label()
    ui.timer(0.1, lambda: state.set_text(f'{world.time:.3f} s ({world.robot.prediction})'))

    click_mode = ui.toggle({'drive': 'Drive to', 'add': 'Add obstacle'}).props('outline clearable')

    def handle_click(msg):
        if msg.click_type != 'dblclick':
            return
        for hit in msg.objects:
            if hit.name == 'ground' and click_mode.value == 'drive':
                target = Point(x=hit.point.x, y=hit.point.y)
                runtime.automator.add(drive_to(world, runtime.esp, target))
                runtime.resume()
    with ui.row():
        with ui.scene(640, 480, on_click=handle_click) as scene:
            outline = list(map(list, world.robot.shape.outline))
            robot = scene.extrusion(outline, world.robot.shape.height).material('#4488ff')
            ui.timer(0.05, lambda: robot
                     .move(world.robot.prediction.x, world.robot.prediction.y)
                     .rotate(0, 0, world.robot.prediction.yaw) and False)
        Joystick(size=50, color='blue', steerer=runtime.steerer)

    with ui.row():
        ui.button(on_click=runtime.resume).props('outline icon=play_arrow') \
            .bind_visibility_from(world.state, value=WorldState.PAUSED)
        ui.button(on_click=lambda: asyncio.create_task(runtime.pause())).props('outline icon=pause') \
            .bind_visibility_from(world.state, value=WorldState.RUNNING)
        ui.button('restart rosys', on_click=lambda: os.utime('main.py')).props('outline')

ui.on_startup(runtime.run())
ui.on_shutdown(runtime.stop())

app.routes.insert(0, starlette.routing.Route(
    '/world', lambda *_: starlette.responses.Response(content=world.json(exclude={'image_data'}), media_type='text/json')))

ui.run(title="obstacles")
