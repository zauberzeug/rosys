#!/usr/bin/env python3
from nicegui import ui

from rosys.vision import UsbCameraProviderSimulation, camera_provider

camera_provider = UsbCameraProviderSimulation()
camera_provider.add_camera(camera_provider.create_calibrated('test_cam', width=800, height=600))


def refresh() -> None:
    for uid, camera in camera_provider.cameras.items():
        if uid not in feeds:
            feeds[uid] = ui.image()
        feeds[uid].set_source(camera_provider.get_latest_image_url(camera))


feeds = {}
ui.timer(0.3, refresh)

ui.run(title='RoSys')
