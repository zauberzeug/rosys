#!/usr/bin/env python3
"""Interactive demo for a GMSL2/FPD-Link camera on an NVIDIA Jetson.

Shows the live Argus stream with controls for the headline capability: a settable
exposure. Toggle auto exposure off and raise the exposure (lowering the fps) to
get a long-exposure image; the hardware ISP still handles color and white balance.
"""
from nicegui import ui

import rosys

SENSOR_ID = 0  # the Argus sensor-id (GMSL port); adjust to the connected camera

camera = rosys.vision.GmslCamera(id=f'gmsl-{SENSOR_ID}', sensor_id=SENSOR_ID)

# Pin a long manual exposure and a matching low fps (the streak-capture preset).
LONG_EXPOSURE_PRESET = {'auto_exposure': False, 'exposure': 0.25, 'fps': 4}

image = ui.interactive_image()

with ui.row().classes('items-center gap-4'):
    ui.label().bind_text_from(camera, 'is_connected', lambda c: '🟢 connected' if c else '🔴 disconnected')

    auto = ui.switch('Auto exposure') \
        .bind_value_from(camera, 'parameters', lambda p: p['auto_exposure'])
    auto.on_value_change(lambda e: camera.set_parameters({'auto_exposure': e.value}))

    with ui.column().classes('gap-0'):
        ui.label('Exposure (s)')
        ui.slider(min=0.000015, max=0.25, step=0.000015,
                  on_change=lambda e: camera.set_parameters({'exposure': e.value})) \
            .props('label-always') \
            .bind_value_from(camera, 'parameters', lambda p: p['exposure'])

    with ui.column().classes('gap-0'):
        ui.label('Gain')
        ui.slider(min=1.0, max=9.0, step=0.1,
                  on_change=lambda e: camera.set_parameters({'auto_gain': False, 'gain': e.value})) \
            .props('label-always') \
            .bind_value_from(camera, 'parameters', lambda p: p['gain'])

    ui.number('fps', min=1, max=100, step=1,
              on_change=lambda e: camera.set_parameters({'fps': int(e.value)})) \
        .bind_value_from(camera, 'parameters', lambda p: p['fps'])

    ui.button('Long-exposure preset', on_click=lambda: camera.set_parameters(LONG_EXPOSURE_PRESET)).props('outline')

ui.timer(0.1, lambda: image.set_source(camera.get_latest_image_url()))

ui.run(title='RoSys GMSL camera')
