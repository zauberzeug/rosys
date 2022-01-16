#!/usr/bin/env python3
from nicegui import ui
import rosys
import rosys.ui

# setup
runtime = rosys.Runtime()
rosys.ui.configure(ui, runtime)


def refresh():
    for uid, camera in runtime.world.cameras.items():
        if uid not in feeds:
            with ui.card().tight().style('width:30em;'):
                feeds[uid] = ui.image()
        feeds[uid].set_source(camera.latest_frame_uri)


# refresh timer
feeds = {}
ui.timer(0.3, refresh)

ui.run(title='RoSys', port=8080)
