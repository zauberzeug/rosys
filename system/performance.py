#!/usr/bin/env python3

import asyncio
import time

loop = asyncio.get_event_loop()


async def sleep():
    while True:
        await asyncio.sleep(0.01)

def main():
    print("starting")
    loop.run_until_complete(sleep()) 

