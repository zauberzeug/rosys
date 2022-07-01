#!/usr/bin/env python3
from nicegui import ui
from rosys import runtime
from rosys.actors import CameraServer, UsbCameraCapture, UsbCameraSimulator, camera_provider

# setup
if UsbCameraCapture.is_operable():
    camera_provider = UsbCameraCapture()
else:
    camera_provider = UsbCameraSimulator()
    camera_provider.add_camera(camera_provider.create_calibrated('test_cam', width=800, height=600))
CameraServer(camera_provider)


async def refresh() -> None:
    for uid, camera in camera_provider.cameras.items():
        if uid not in feeds:
            feeds[uid] = ui.interactive_image('', cross=False)
        await feeds[uid].set_source(camera.latest_image_uri)


# ui
feeds = {}
ui.timer(0.3, refresh)

# start
ui.on_startup(runtime.startup)
ui.on_shutdown(runtime.shutdown)
ui.run(title='RoSys')
