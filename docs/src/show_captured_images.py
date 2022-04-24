#!/usr/bin/env python3
import rosys
import rosys.ui
from nicegui import ui
from rosys.actors import UsbCameraSimulator

# setup
runtime = rosys.Runtime()
runtime.with_usb_cameras()
rosys.ui.configure(ui, runtime)


async def refresh():
    for uid, camera in runtime.world.usb_cameras.items():
        if uid not in feeds:
            feeds[uid] = ui.interactive_image('', cross=False)
        await feeds[uid].set_source(camera.latest_image_uri)


# refresh timer
feeds = {}
ui.timer(0.3, refresh)

camsim = runtime.get_actor(UsbCameraSimulator)
if camsim is not None and 'testcam' not in runtime.world.usb_cameras:
    runtime.world.usb_cameras['testcam'] = \
        camsim.create_calibrated('testcam', width=600, height=800)

ui.run(title='RoSys', port=8080)
