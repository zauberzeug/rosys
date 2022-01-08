#!/usr/bin/env python3
from nicegui import ui
import rosys
import rosys.ui
from rosys.world import Mode, World
import icecream
icecream.install()

# setup
runtime = rosys.Runtime(world=World(mode=Mode.SIMULATION))
rosys.ui.configure(ui, runtime)
feeds = {}


def refresh():
    for uid, camera in runtime.world.cameras.items():
        if uid not in feeds:
            with ui.card().tight().style('width:30em;'):
                feeds[uid] = ui.image()
        feeds[uid].set_source(camera.latest_frame_uri)


ui.timer(1, refresh)

# start
ui.run(title='RoSys', port=80)
