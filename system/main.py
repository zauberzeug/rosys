from fastapi import FastAPI
from fastapi.encoders import jsonable_encoder
from fastapi_socketio import SocketManager
import asyncio
import task_logger
import uvicorn
import os
from numpy import deg2rad as rad
from runtime import Runtime
from world.mode import Mode
from world.world import World

app = FastAPI()
sio = SocketManager(app=app)

has_esp = os.stat('/dev/esp').st_gid > 0
runtime = Runtime(Mode.REAL if has_esp else Mode.SIMULATION)


@sio.on('drive_power')
async def on_drive_power(_, data):
    await runtime.esp.power(data['left'], data['right'])


@sio.on('task')
async def on_task(_, data):

    async def square(linear_speed=0.5, angular_speed=rad(45)):
        while runtime.world.robot.pose.x < 2:
            await runtime.esp.drive(linear_speed, 0)
            await asyncio.sleep(0.01)
        while runtime.world.robot.pose.yaw < rad(90):
            await runtime.esp.drive(0, angular_speed)
            await asyncio.sleep(0.01)
        while runtime.world.robot.pose.y < 2:
            await runtime.esp.drive(linear_speed, 0)
            await asyncio.sleep(0.01)
        while runtime.world.robot.pose.yaw < rad(180):
            await runtime.esp.drive(0, angular_speed)
            await asyncio.sleep(0.01)
        while runtime.world.robot.pose.x > 0:
            await runtime.esp.drive(linear_speed, 0)
            await asyncio.sleep(0.01)
        while runtime.world.robot.pose.yaw < rad(270):
            await runtime.esp.drive(0, angular_speed)
            await asyncio.sleep(0.01)
        while runtime.world.robot.pose.y > 0:
            await runtime.esp.drive(linear_speed, 0)
            await asyncio.sleep(0.01)
        await runtime.esp.drive(0, 0)

    if data == "square":
        task_logger.create_task(square())


async def do_updates():
    while True:
        await sio.emit('world', jsonable_encoder(runtime.world))
        await asyncio.sleep(0.1)

tasks = []


@app.on_event("startup")
async def startup():
    loop = asyncio.get_event_loop()
    loop.set_debug(True)

    tasks.append(task_logger.create_task(runtime.run()))
    tasks.append(task_logger.create_task(do_updates()))


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


if __name__ == "__main__":
    uvicorn.run(
        "main:app", host="0.0.0.0", port=7000,
        reload=True, lifespan='on', forwarded_allow_ips='*', proxy_headers=True
    )
