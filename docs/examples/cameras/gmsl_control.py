#!/usr/bin/env python3
"""Interactive demo for a GMSL2/FPD-Link camera on an NVIDIA Jetson.

Shows the live Argus stream with controls for the headline capability: a settable
exposure. Toggle auto exposure off and raise the exposure (lowering the fps) to
get a long-exposure image; the hardware ISP still handles color and white balance.

Run this on the Jetson (where `nvarguscamerasrc` lives) and open the page from
another machine. Because GMSL camera parameters are fixed when the GStreamer
pipeline starts, every change restarts the pipeline -- so the controls below set
values on user interaction only (they are intentionally not bound back to the
camera, which would restart the pipeline on every update).
"""
import multiprocessing

from nicegui import ui

import rosys

SENSOR_ID = 0  # the Argus sensor-id (GMSL port); adjust to the connected camera

camera = rosys.vision.GmslCamera(id=f'gmsl-{SENSOR_ID}', sensor_id=SENSOR_ID)

image = ui.interactive_image()
ui.timer(0.1, lambda: image.set_source(camera.get_latest_image_url()))

with ui.row().classes('items-center gap-4'):
    ui.label().bind_text_from(camera, 'is_connected', lambda c: '🟢 connected' if c else '🔴 disconnected')

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

if __name__ in {'__main__', '__mp_main__'}:
    multiprocessing.set_start_method('spawn', force=True)
    ui.run(title='RoSys GMSL camera', reload=False)
