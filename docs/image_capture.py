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
    for id in runtime.world.cameras.keys():
        if id not in feeds:
            with ui.card().tight().style('width:30em;'):
                feeds[id] = ui.image()
        feeds[id].set_source(f'camera/{id}/latest?{runtime.world.time}')


ui.timer(0.3, refresh)

# start
ui.run(title='RoSys', port=80)
