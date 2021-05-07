from icecream import ic, install
from fastapi import FastAPI, Response
from fastapi.encoders import jsonable_encoder
from fastapi_socketio import SocketManager
from typing import Set
import asyncio
import task_logger
import uvicorn
import os
from runtime import Runtime
from world.mode import Mode
from world.world import World
from world.pose import Pose
from automations.navigation.spline import Spline
from automations.square import square
from automations.spline import spline

import uvloop
uvloop.install()

ic("Installing icecream...")
install()


app = FastAPI()
sio = SocketManager(app=app)

has_esp = os.stat('/dev/esp').st_gid > 0
runtime = Runtime(Mode.REAL if has_esp else Mode.SIMULATION)


@sio.on('drive_power')
async def on_drive_power(_, data):
    await runtime.esp.power(data['left'], data['right'])


@sio.on('task')
async def on_task(_, data):

    if data == "square":
        runtime.automator.add(square(runtime.world, runtime.esp))

    if data == "spline":
        s = Spline(Pose(x=0, y=0, yaw=0), Pose(x=2, y=1, yaw=0))
        runtime.automator.add(spline(s, runtime.world, runtime.esp))


@sio.on('pause')
async def pause(*_):

    await runtime.pause()


@sio.on('resume')
def resume(*_):

    runtime.resume()


async def broadcast(interval: float, topic, *, exclude: Set[str]):
    while True:
        await sio.emit(topic, jsonable_encoder(runtime.world, exclude=exclude))
        await asyncio.sleep(interval)


tasks = []


@app.on_event("startup")
async def startup():

    loop = asyncio.get_event_loop()
    # loop.set_debug(True) # NOTE this makes execution slow -- use with care

    broadcasts = [
        broadcast(1.0, 'world', exclude={'image_data'}),
        broadcast(0.1, 'world_part', exclude={'image_data', 'mode', 'cameras', 'images'}),
    ]

    tasks.append(task_logger.create_task(runtime.run()))
    tasks.extend([task_logger.create_task(b) for b in broadcasts])


@app.on_event("shutdown")
async def shutdown():
    for task in tasks:
        task.cancel()


@app.get("/")
def main():
    return {"status": "hello, I'm the robot system!", 'world': jsonable_encoder(runtime.world)}


@app.get("/world", response_model=World)
def get_world():
    return jsonable_encoder(runtime.world)


@app.get("/image/{id}")
def get_image(id):
    print("IMAGE!")
    return Response(content=runtime.world.image_data[id], media_type='image/jpeg')


if __name__ == "__main__":
    uvicorn.run(
        "main:app", host="0.0.0.0", port=7000,
        reload=True, lifespan='on', forwarded_allow_ips='*', proxy_headers=True
    )
