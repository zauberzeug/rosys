#!/usr/bin/env python3
from nicegui import ui

import rosys
from rosys.vision import Camera

rtsp_camera_provider = rosys.vision.RtspCameraProvider()
usb_camera_provider = rosys.vision.UsbCameraProvider()
simulated_camera_provider = rosys.vision.SimulatedCameraProvider()
simulated_camera_provider2 = rosys.vision.SimulatedCameraProvider()


def create_ui(camera_provider):
    for uid, camera in camera_provider.cameras.items():
        if uid not in streams:
            with camera_grid:
                with ui.card().tight():
                    streams[uid] = ui.interactive_image()
                    ui.label(uid).classes('m-2')
                    with ui.row():
                        ui.switch('stream').bind_value(camera, 'streaming')
                        ui.button('capture', on_click=camera.capture_image)
        streams[uid].set_source(camera.get_latest_image_url())


def refresh() -> None:
    for provider in [rtsp_camera_provider, usb_camera_provider, simulated_camera_provider]:
        create_ui(provider)


streams: dict[str, ui.interactive_image] = {}
camera_grid = ui.row()
ui.timer(0.01, refresh)

simulated_camera_provider.add_cameras(1)
simulated_camera_provider2.add_cameras(1)

ui.run(title='RoSys', port=8080)
