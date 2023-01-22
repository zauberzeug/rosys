#!/usr/bin/env python3
import logging

import icecream
from nicegui import ui

from rosys.vision import RtspCameraProviderHardware, camera_provider

logging.basicConfig(level=logging.INFO)


icecream.install()
camera_provider = RtspCameraProviderHardware()


def refresh() -> None:
    for uid, camera in camera_provider.cameras.items():
        if uid not in feeds:
            with cameras:
                feeds[uid] = ui.interactive_image()
                ui.label(uid)
                ic('creating new feed', uid)
        url = camera_provider.get_latest_image_url(camera)
        feeds[uid].set_source(url)


feeds = {}
cameras = ui.row()
ui.timer(0.01, refresh)

ui.run(title='RoSys')
