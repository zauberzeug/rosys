#!/usr/bin/env python3
import rosys
import rosys.ui
from nicegui import ui
from rosys.world import Camera

# setup
rosys.ui.configure(ui, rosys.Runtime())


async def add_main_camera(camera: Camera):
    camera_card.clear()  # remove "seeking cam" label
    with camera_card:
        maincam = ui.image()
        ui.timer(1, lambda: maincam.set_source(camera.latest_image_uri))
    rosys.event.unregister(rosys.event.Id.NEW_CAMERA, add_main_camera)  # we only show the first cam

with ui.card().tight().style('width:30em;') as camera_card:
    ui.label('seeking main camera').style('margin:1em')
rosys.event.register(rosys.event.Id.NEW_CAMERA, add_main_camera)

with ui.card().tight().style('width:30em;'):
    with ui.row():
        with ui.card().tight():
            rosys.ui.joystick()
            rosys.ui.keyboard_control()
        ui.markdown('steer with joystick on the left<br>or SHIFT + arrow keys') \
            .style('margin:2em;text-align:center')

ui.run(title='RoSys', port=8080)
