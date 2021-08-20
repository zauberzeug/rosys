#!/usr/bin/env python3
import asyncio
import logging
import os
import socket
import starlette
from typing import List

from rosys import task_logger
from rosys.runtime import Runtime
from rosys.ui.joystick import Joystick
from rosys.ui.three import Three
from rosys.world.marker import Marker
from rosys.world.mode import Mode
from rosys.world.robot import Robot
from rosys.world.world import WorldState, World
from rosys.automations.square import drive_square
from nicegui import app, ui
import log_configuration

log_configuration.setup()
logger = logging.getLogger('hello_bot')

world = World(
    mode=Mode.REAL if os.path.exists('/dev/esp') and os.stat('/dev/esp').st_gid > 0 else Mode.SIMULATION,
    state=WorldState.PAUSED,
    robot=Robot(),
    marker=Marker.four_points(0.24, 0.26, 0.41),
)

runtime = Runtime(world)

with ui.column().classes('w-full items-stretch'):
    with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap'):
        steering_card = ui.card()
        actor_card = ui.card().style('width:20em')
        parameter_card = ui.card().classes('flex-grow')
    debug_card = ui.card().classes('items-center')

with steering_card:
    state = ui.label()
    ui.timer(0.1, lambda: state.set_text(f'''
        {world.time:.3f} s
        {world.state.name}
        (x={world.robot.prediction.x:.3f},
        y={world.robot.prediction.y:.3f},
        {world.robot.battery:.1f}V,
        {world.robot.temperature:.1f}C)
    '''))
    with ui.row():
        Joystick(size=50, color='blue', steerer=runtime.steerer)

        def start():
            runtime.automator.add(drive_square(runtime.world, runtime.esp))
            runtime.resume()
        ui.button('drive square', on_click=start)

    def update_three():
        need_updates = [
            three.set_robot('prediction', world.robot.prediction, world.robot.shape),
            three.update_carrot(world.carrot)
        ]
        return not all(n == False for n in need_updates)
    three = Three()
    ui.timer(0.05, update_three)

with actor_card:

    ui.label('Actors')

    with ui.column().style('gap:0'):
        actor_labels = {actor: ui.label()for actor in runtime.actors}
        ui.timer(1.0, lambda: [label.set_text(str(actor)) for actor, label in actor_labels.items()])

with parameter_card:

    ui.label('Parameters')

    ui.number('linear speed limit [m/s]').bind_value(world.robot.parameters.linear_speed_limit)
    ui.number('angular speed limit [rad/s]').bind_value(world.robot.parameters.angular_speed_limit)
    ui.number('carrot distance [m]').bind_value(world.robot.parameters.carrot_distance)

with debug_card:
    with ui.row():
        ui.button('restart rosys', on_click=lambda: os.utime('main.py'))

        async def configure():
            for line in [
                'esp erase',
                '+new bluetooth bt ESP_Z18',
                '+new drive drive roboclaw,128,38400',
                '+set drive.mPerTick=0.00001110',
                '+set drive.width=0.45',
                '+new led nozzle MCP_B7',
                '+set nozzle.interval=0.1',
                '+set nozzle.duty=0.999',  # NOTE: repeat=0 does not really work for 100 % duty cycle
                '+set nozzle.repeat=0',
                '+set esp.outputModules=drive',
                '+esp unmute',
                '+set esp.ready=1',
                '+set esp.24v=1',
                'esp restart',
            ]:
                runtime.esp.send(line)
                await asyncio.sleep(0.1)
        ui.button('Configure', on_click=lambda: task_logger.create_task(configure()))
        ui.button('Restart Motor Controller', on_click=lambda: runtime.esp.send('esp restart'))

        offset = ui.label()
    ui.timer(0.1, lambda: offset.set_text(f'Clock offset: {world.robot.clock_offset or 0:.3f} s'))

ui.on_startup(runtime.run())
ui.on_shutdown(runtime.stop())


def get_world(request, **kwargs):

    return starlette.responses.Response(content=world.json(exclude={'image_data'}), media_type='text/json')


app.routes.insert(0, starlette.routing.Route('/world', get_world))

ui.run(title="RoSys")
