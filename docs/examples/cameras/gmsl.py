#!/usr/bin/env python3
from nicegui import ui

import rosys

# A GMSL2/FPD-Link camera attached to an NVIDIA Jetson (e.g. a global-shutter camera on a Jetson GMSL carrier board).
# Frames are captured through NVIDIA's Argus stack, so the hardware ISP handles debayering and white balance.
# The camera is identified by its Argus sensor-id (the GMSL port on the board).
camera_provider = rosys.vision.GmslCameraProvider()
camera = rosys.vision.GmslCamera(id='gmsl-0', sensor_id=0)
camera_provider.add_camera(camera)

# For long-exposure capture (e.g. a material-flow streak approach), pin a long manual
# exposure and lower the fps so the frame period can accommodate it -- ISP correction stays on:
#   await camera.set_parameters({'auto_exposure': False, 'exposure': 0.25, 'fps': 4})

image = ui.interactive_image()
ui.timer(0.1, lambda: image.set_source(camera.get_latest_image_url()))

ui.run(title='RoSys')
