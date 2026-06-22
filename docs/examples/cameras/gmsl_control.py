#!/usr/bin/env python3
"""Interactive demo for a GMSL2/FPD-Link camera on an NVIDIA Jetson.

Shows the live Argus stream with controls for the headline capability: a settable
exposure. Toggle auto exposure off and raise the exposure (lowering the fps) to
get a long-exposure image; the hardware ISP still handles color and white balance.

The UI lives in a ``@ui.page`` function (not at global scope): NiceGUI re-runs a
global-scope script on every page load, which would spawn a new Argus pipeline fighting
the camera's single session each time. Because GMSL parameters are fixed when the pipeline
starts, each change restarts the pipeline, so the controls set values on user interaction
only (they are not bound back to the camera).
"""
from nicegui import ui

import rosys

SENSOR_ID = 0  # the Argus sensor-id (GMSL port); adjust to the connected camera

camera = rosys.vision.GmslCamera(id=f'gmsl-{SENSOR_ID}', sensor_id=SENSOR_ID)


@ui.page('/')
def index() -> None:
    image = ui.interactive_image()
    ui.timer(0.1, lambda: image.set_source(camera.get_latest_image_url()))

    with ui.row().classes('items-center gap-4'):
        ui.label().bind_text_from(camera, 'is_connected', lambda c: 'connected' if c else 'disconnected')

        ui.switch('Auto exposure', value=True,
                  on_change=lambda e: camera.set_parameters({'auto_exposure': e.value}))

        with ui.column().classes('gap-0'):
            ui.label('Exposure (s)')
            ui.slider(min=0.000015, max=0.25, step=0.000015, value=0.01,
                      on_change=lambda e: camera.set_parameters({'auto_exposure': False, 'exposure': e.value})) \
                .props('label-always')

        with ui.column().classes('gap-0'):
            ui.label('Gain')
            ui.slider(min=1.0, max=9.0, step=0.1, value=1.0,
                      on_change=lambda e: camera.set_parameters({'auto_gain': False, 'gain': e.value})) \
                .props('label-always')

        ui.number('fps', value=30, min=1, max=100, step=1,
                  on_change=lambda e: camera.set_parameters({'fps': int(e.value)}))

        ui.button('Long-exposure preset',
                  on_click=lambda: camera.set_parameters({'auto_exposure': False, 'exposure': 0.25, 'fps': 4})) \
            .props('outline')


ui.run(title='RoSys GMSL camera')
