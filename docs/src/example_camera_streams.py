#!/usr/bin/env python3
import logging

import icecream

if True:
    from engineio.payload import Payload
    Payload.max_decode_packets = 500

from nicegui import ui

import rosys

logging.basicConfig(level=logging.INFO)


icecream.install()
camera_provider = rosys.vision.RtspCameraProviderHardware()


def refresh() -> None:
    for uid, camera in camera_provider.cameras.items():
        if uid not in feeds:
            with cameras:
                with ui.card().tight():
                    feeds[uid] = ui.interactive_image()
                    ui.label(uid).classes('m-2')
                ic('creating new feed', uid)
        url = camera_provider.get_latest_image_url(camera)
        feeds[uid].set_source(url)


feeds = {}
warnings = rosys.analysis.asyncio_warnings.AsyncioWarnings()
cameras = ui.row()
ui.timer(0.01, refresh)
rosys.on_startup(warnings.activate)

ui.run(title='RoSys')
