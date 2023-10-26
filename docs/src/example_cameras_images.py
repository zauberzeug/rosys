#!/usr/bin/env python3
from nicegui import ui

from rosys.vision import ImageSize, SimulatedCamera

camera = SimulatedCamera(id='test_cam', resolution=ImageSize(width=800, height=600))

image = ui.interactive_image()
ui.timer(0.3, lambda: image.set_source(camera.get_latest_image_url()))

ui.run(title='RoSys')
