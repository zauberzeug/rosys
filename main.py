#!/usr/bin/env python3
import asyncio
from typing import List
from nicegui import app, ui
import os
import starlette
import socket
import logging
from rosys import task_logger
from rosys.runtime import Runtime
from rosys.world.mode import Mode
from rosys.world.marker import Marker
from rosys.automations.draw import draw_path
from rosys.ui.joystick import Joystick
from rosys.ui.three import Three
import drawings

import icecream
icecream.install()

logging.basicConfig(level=logging.INFO)

has_esp = os.path.exists('/dev/esp') and os.stat('/dev/esp').st_gid > 0
runtime = Runtime(Mode.REAL if has_esp else Mode.SIMULATION)
# runtime = Runtime(Mode.SIMULATION)

if socket.gethostname() in ['z18', 'docker']:
    logging.warning(f'using config for z18')
    runtime.world.robot.shape.outline = [(0.41, 0.205), (0.5, 0), (0.41, -0.205), (-0.115, -0.205), (-0.115, 0.205)]
    runtime.world.robot.shape.point_of_interest.x = -0.115
    runtime.world.robot.shape.height = 0.19
    runtime.world.marker = Marker.four_points(0.15, 0.14, 0.19)
else:
    logging.warning(f'unknown robot host "{socket.gethostname()}" using default settings')


with ui.column().classes('w-full items-stretch'):
    with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap'):
        svg_card = ui.card()
        steering_card = ui.card()
        with ui.column().classes('flex-grow items-stretch justify-items-stretch'):
            image_card = ui.card().classes('flex-grow')
            camera_card = ui.card()
    with ui.row().classes('items-stretch justify-items-stretch'):
        actor_card = ui.card()
        parameter_card = ui.card().classes('flex-grow')
        controller_card = ui.card()

with steering_card:

    state = ui.label()
    ui.timer(0.1, lambda: state.set_text(f'''
        {runtime.world.time:.3f} s
        {runtime.world.state.name}
        (x={runtime.world.robot.prediction.x:.3f},
        y={runtime.world.robot.prediction.y:.3f})
    '''))

    with ui.row():
        Joystick(size=50, color='blue', steerer=runtime.steerer)
        ui.switch('Nozzle', on_change=lambda e: runtime.esp.send(f'nozzle {"on" if e.value else "off"}'))

    def update_three():
        need_updates = [
            three.set_robot('prediction', '#6E93D6', runtime.world.robot.prediction),
            three.set_robot('detection', '#AEC0E2', runtime.world.robot.detection),
            three.update_path(runtime.world.path),
        ]
        return not all(n == False for n in need_updates)
    three = Three(robot_shape=runtime.world.robot.shape)
    ui.timer(0.05, update_three)

with svg_card:

    def set_image_source():
        image.set_source(f'drawings/default?t={runtime.world.time}')

    def set_path():
        runtime.world.path = drawings.load(runtime.world.robot.prediction, size.value)

    def upload(files: List[bytearray]):
        drawings.store(files[0])
        set_image_source()
        set_path()

    ui.upload(on_upload=upload)
    image = ui.image(source=None)

    with ui.row().classes('items-end'):

        def start():
            runtime.automator.add(draw_path(runtime.world, runtime.esp))
            runtime.resume()

        def stop():
            task_logger.create_task(runtime.pause())

        size = ui.number('Size [m]', value=2.0, on_change=set_path)
        ui.button('Start', on_click=start).props('icon=play_arrow')
        ui.button('Stop', on_click=stop).props('icon=stop')

    set_image_source()
    set_path()

with image_card:

    with ui.row():

        def download():
            runtime.world.download_queue = list(runtime.world.cameras.keys())

        ui.button('Download images', on_click=download)
        download_timer = ui.timer(0.1, download)
        ui.checkbox('Track').bind_value_to(download_timer.active)

    with ui.image().style('width:20em') as ui_image:
        ui_image.id = None
        svg = ui.svg().style('background:transparent')

    def update_camera_images():

        three.update_images(runtime.world.images, runtime.world.cameras)

        if not any(runtime.world.images):
            return False

        image = runtime.world.images[-1]

        data = runtime.world.image_data.get(image.id)
        if data is None:
            return False

        if image.detections is None:
            return False

        if ui_image.id == image.id:
            return False

        ui_image.source = f'imagedata/{image.id}'
        ui_image.id = image.id
        ic(image.detections)
        svg_content = '<svg viewBox="0 0 1600 1200" width="100%" height="100%" xmlns="http://www.w3.org/2000/svg">'
        for d in image.detections:
            svg_content += f'<rect x="{d.x}" y="{d.y}" width="{d.width}" height="{d.height}" stroke="red" fill="red" fill-opacity="10%" />'
            c = {
                'dirt': 'D',
                'robot': 'R',
                'person': 'P',
                'marker_vorne': 'v',
                'marker_mitte': 'm',
                'marker_hinten_links': 'l',
                'marker_hinten_rechts': 'r',
            }.get(d.category_name) or ''
            svg_content += f'<text x="{d.x+2}" y="{d.y+d.height-2}" fill="red">{c}</text>'
        svg_content += '</svg>'
        svg.content = svg_content

    ui.timer(1.0, update_camera_images)

with camera_card:

    ui.label('Cameras')

    cams = ui.label()
    ui.timer(1, lambda: cams.set_text('Clock offsets: ' + str({
        k: v.synchronization.offset if v.synchronization is not None else None
        for k, v in runtime.world.cameras.items()
    })))

    def set_height(height):
        for camera in runtime.world.cameras.values():
            try:
                camera.calibration.extrinsics.translation[2] = height
                camera.projection = None
            except AttributeError:
                pass

    ui.number('Height [m]', on_change=lambda e: set_height(e.value))

    with ui.row():
        def clear_calibrations():
            for camera in runtime.world.cameras.values():
                camera.calibration = None
                camera.projection = None

        def clear_cameras():
            runtime.world.cameras = {}
        ui.button('Clear calibrations', on_click=clear_calibrations)
        ui.button('Clear cameras', on_click=clear_cameras)

with actor_card:

    ui.label('Actors')

    with ui.column().style('gap:0'):
        actor_labels = {actor: ui.label()for actor in runtime.actors}
        ui.timer(1.0, lambda: [label.set_text(str(actor)) for actor, label in actor_labels.items()])

with parameter_card:

    ui.label('Parameters')

    ui.number('linear speed limit [m/s]').bind_value(runtime.world.robot.parameters.linear_speed_limit)
    ui.number('angular speed limit [rad/s]').bind_value(runtime.world.robot.parameters.angular_speed_limit)
    ui.number('carrot distance [m]').bind_value(runtime.world.robot.parameters.carrot_distance)

with controller_card:

    ui.label('Controller')

    async def configure():
        for line in [
            'esp erase',
            '+new bluetooth bt ESP_Z18',
            '+new drive drive roboclaw,128,38400',
            '+set drive.mPerTick=0.00001110',
            '+set drive.width=0.45',
            '+new led nozzle MCP_B7',
            '+set esp.outputModules=drive',
            '+esp unmute',
            '+set esp.ready=1',
            '+set esp.24v=1',
            'esp restart',
        ]:
            runtime.esp.send(line)
            await asyncio.sleep(0.1)
    ui.button('Configure', on_click=lambda: task_logger.create_task(configure()))
    ui.button('Restart', on_click=lambda: runtime.esp.send('esp restart'))

    offset = ui.label()
    ui.timer(0.1, lambda: offset.set_text(f'Clock offset: {runtime.world.robot.clock_offset or 0:.3f} s'))

ui.on_startup(runtime.run())
ui.on_shutdown(runtime.stop())


def get_image_data(request, **kwargs):

    id = request.path_params['id']
    return starlette.responses.Response(content=runtime.world.image_data[id], media_type='image/jpeg')


def get_drawing(request, **kwargs):

    return starlette.responses.FileResponse(f'{drawings.path}/{request.path_params["id"]}.svg')


def get_world(request, **kwargs):

    return starlette.responses.Response(content=runtime.world.json(exclude={'image_data'}), media_type='text/json')


app.routes.insert(0, starlette.routing.Route('/imagedata/{id}', get_image_data))
app.routes.insert(0, starlette.routing.Route('/drawings/{id}', get_drawing))
app.routes.insert(0, starlette.routing.Route('/world', get_world))

ui.run(title="RoSys")
