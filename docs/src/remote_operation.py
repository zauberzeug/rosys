#!/usr/bin/env python3
from nicegui import ui
import rosys
import rosys.ui
from rosys.world import Mode, World
from rosys.world import Camera

# setup
runtime = rosys.Runtime(world=World(mode=Mode.SIMULATION))
rosys.ui.configure(ui, runtime)


async def add_main_camera(camera: Camera):
    camera_card.clear()  # remove label
    with camera_card:
        maincam = ui.image()
        ui.timer(1, lambda: maincam.set_source(camera.latest_frame_uri))
    rosys.event.unregister(rosys.event.Id.NEW_CAMERA, add_main_camera)


with ui.card().tight().style('width:30em;') as camera_card:
    ui.label('seeking main camera').style('margin:1em')
rosys.event.register(rosys.event.Id.NEW_CAMERA, add_main_camera)

ui.run(title='RoSys', port=80, show=False)
