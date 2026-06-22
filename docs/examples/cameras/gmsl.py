#!/usr/bin/env python3
"""Minimal live view of a GMSL2/FPD-Link camera on an NVIDIA Jetson.

Frames are captured through NVIDIA's Argus stack, so the hardware ISP handles debayering
and white balance. The camera is identified by its Argus sensor-id (the GMSL port).

The UI is defined inside a ``@ui.page`` function rather than at global scope: NiceGUI
re-runs a global-scope script on every page load, which would create a new camera (and a
new Argus pipeline fighting the camera's single session) each time. With a page function
the script runs once, so the single top-level camera is created and connected once.
"""
from nicegui import ui

import rosys

SENSOR_ID = 0  # the Argus sensor-id (GMSL port); adjust to the connected camera

camera = rosys.vision.GmslCamera(id=f'gmsl-{SENSOR_ID}', sensor_id=SENSOR_ID)


@ui.page('/')
def index() -> None:
    image = ui.interactive_image()
    ui.timer(0.1, lambda: image.set_source(camera.get_latest_image_url()))


ui.run(title='RoSys')
