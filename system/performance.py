#!/usr/bin/env python3

import asyncio
import time
import os
from runtime import Runtime
from world.mode import Mode


has_esp = os.stat('/dev/esp').st_gid > 0
runtime = Runtime(Mode.REAL if has_esp else Mode.SIMULATION)

loop = asyncio.get_event_loop()

async def sleep():
    while True:
        await asyncio.sleep(0.01)

def main():
    loop = asyncio.get_event_loop()
    print("starting")
    loop.run_until_complete(runtime.run())
    #loop.run_until_complete(sleep()) 

