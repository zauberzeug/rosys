#!/usr/bin/env python3
from nicegui import ui
import rosys
import rosys.ui

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

ui.run(title='RoSys', port=8080)
