#!/usr/bin/env python3
import logging

import icecream

if True:
    from engineio.payload import Payload
    Payload.max_decode_packets = 500

from nicegui import ui

import rosys

camera_provider = rosys.vision.RtspCameraProviderHardware()


def refresh() -> None:
    for uid, camera in camera_provider.cameras.items():
        if uid not in streams:
            with camera_grid:
                with ui.card().tight():
                    streams[uid] = ui.interactive_image()
                    ui.label(uid).classes('m-2')
        streams[uid].set_source(camera_provider.get_latest_image_url(camera))


streams = {}
camera_grid = ui.row()
ui.timer(0.01, refresh)

ui.run(title='RoSys')
