#!/usr/bin/env python3
import icecream
from nicegui import ui

from rosys.vision import RtspCameraProviderHardware, camera_provider

icecream.install()
camera_provider = RtspCameraProviderHardware()


def refresh() -> None:
    for uid, camera in camera_provider.cameras.items():
        if uid not in feeds:
            feeds[uid] = ui.interactive_image()
        feeds[uid].set_source(camera_provider.get_latest_image_url(camera))


feeds = {}
ui.timer(0.01, refresh)

ui.run(title='RoSys')
