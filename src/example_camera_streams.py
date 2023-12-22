#!/usr/bin/env python3
from nicegui import ui

import rosys

camera_provider = rosys.vision.RtspCameraProvider()


def refresh() -> None:
    for uid, camera in camera_provider.cameras.items():
        if uid not in streams:
            with camera_grid:
                with ui.card().tight():
                    streams[uid] = ui.interactive_image()
                    ui.label(uid).classes('m-2')
        streams[uid].set_source(camera.get_latest_image_url())


streams: dict[str, ui.interactive_image] = {}
camera_grid = ui.row()
ui.timer(0.01, refresh)

ui.run(title='RoSys')
