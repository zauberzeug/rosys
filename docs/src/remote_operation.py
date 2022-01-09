#!/usr/bin/env python3
import asyncio
from nicegui import ui
import rosys
import rosys.ui
from rosys.world import Mode, World
import icecream
import logging
icecream.install()
# setup
runtime = rosys.Runtime(world=World(mode=Mode.SIMULATION))
rosys.ui.configure(ui, runtime)


async def configure_maincam():
    with ui.card().tight().style('width:30em;') as card:
        ui.label('seeking main camera').style('margin:1em')
    # wait for first cam to appear
    while True:
        if len(runtime.world.cameras) > 0:
            break
        await asyncio.sleep(0.5)
    card.clear()  # remove label
    camera = list(runtime.world.cameras.values())[0]
    with card:
        maincam = ui.image()
        ui.timer(1, lambda: maincam.set_source(camera.latest_frame_uri))

ui.on_startup(configure_maincam())

ui.run(title='RoSys', port=80, show=False)
