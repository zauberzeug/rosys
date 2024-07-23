#!/usr/bin/env python3
import asyncio

from nicegui import ui

from rosys.analysis import track


@track
async def do_A():
    await asyncio.sleep(1)


@track
async def do_B():
    await asyncio.sleep(1)


@track
async def do_something():
    await asyncio.sleep(1)
    for _ in range(3):
        await do_A()
        await do_B()

ui.button('Do something', on_click=do_something)

track.ui()

ui.run()
