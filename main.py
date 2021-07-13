#!/usr/bin/env python3
from typing import List
from nicegui import app, ui
import os
import starlette
from rosys.runtime import Runtime
from rosys.world.mode import Mode
from rosys.automations.draw import draw
from rosys.ui.joystick import Joystick
from rosys.ui.three import Three
import drawings

import icecream
icecream.install()

has_esp = os.path.exists('/dev/esp') and os.stat('/dev/esp').st_gid > 0
runtime = Runtime(Mode.REAL if has_esp else Mode.SIMULATION)
# runtime = Runtime(Mode.SIMULATION)

with ui.card():

    state = ui.label()
    ui.timer(0.1, lambda: state.set_text(f'''
        {runtime.world.time:.3f} s
        (x={runtime.world.robot.prediction.x:.3f},
        y={runtime.world.robot.prediction.y:.3f})
    '''))

    Joystick(size=50, color='blue', steerer=runtime.steerer)

    def update_three():
        need_updates = [
            three.set_robot('prediction', '#6E93D6', runtime.world.robot.prediction),
            three.set_robot('detection', '#AEC0E2', runtime.world.robot.detection),
            three.update_path(runtime.world.path),
        ]
        return not all(n == False for n in need_updates)
    three = Three()
    ui.timer(0.05, update_three)

with ui.card() as svg_card:

    def set_image_source():
        image.set_source(f'/drawings/default?t={runtime.world.time}')

    def upload(files: List[bytearray]):
        drawings.store(files[0])
        set_image_source()

    ui.upload(on_upload=upload)
    image = ui.image(source=None)
    set_image_source()

    def start():
        runtime.world.path = drawings.scale(drawings.load(), 2.0)
        runtime.automator.add(draw(runtime.world, runtime.esp))
    ui.button('Start', on_click=start)

with ui.card().style('width:600px'):

    def download():
        runtime.world.download_queue = list(runtime.world.cameras.keys())
    ui.button('Download images', on_click=download)
    download_timer = ui.timer(0.1, download)
    ui.checkbox('Track').bind_value_to(download_timer.active)

    with ui.image() as ui_image:
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

with ui.card():

    ui.label('Cameras')

    cams = ui.label()
    ui.timer(1, lambda: cams.set_text(f'cams: {list(runtime.world.cameras.keys())}'))

    def set_height(height):
        for camera in runtime.world.cameras.values():
            try:
                camera.calibration.extrinsics.translation[2] = height
                camera.projection = None
            except AttributeError:
                pass

    ui.number('Height [m]', on_change=lambda e: set_height(e.value))

    def clear_calibrations():
        for camera in runtime.world.cameras.values():
            camera.calibration = None

    ui.button('Clear calibrations', on_click=clear_calibrations)

with ui.card():

    ui.label('Actors')
    actors = ui.label()
    ui.timer(1.0, lambda: actors.set_text(", ".join(map(str, runtime.actors))))


ui.on_startup(runtime.run())
ui.on_shutdown(runtime.stop())


def get_image_data(request, **kwargs):

    id = request.path_params['id']
    return starlette.responses.Response(content=runtime.world.image_data[id], media_type='image/jpeg')


def get_drawing(request, **kwargs):

    return starlette.responses.FileResponse(f'{drawings.path}/{request.path_params["id"]}.svg')


app.routes.insert(0, starlette.routing.Route('/imagedata/{id}', get_image_data))
app.routes.insert(0, starlette.routing.Route('/drawings/{id}', get_drawing))

ui.run(title="RoSys")
