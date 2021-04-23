from fastapi import FastAPI
from fastapi_socketio import SocketManager
import asyncio
import task_logger
import uvicorn
import time
from numpy import deg2rad as rad
from world.world import World
from world.robot import Robot
from actors.serial import Serial
from actors.clock import Clock
from actors.odometer import Odometer
from icecream import ic

app = FastAPI()
sio = SocketManager(app=app)

world = World(
    time=time.time(),
    robot=Robot(),
)

serial = Serial(world)
actors = [
    Clock(world),
    serial,
    Odometer(world),
]


@sio.on('drive_power')
async def on_drive_power(_, data):
    await serial.send('drive pw %.3f,%.3f' % (data['left'], data['right']))


@sio.on('task')
async def on_task(_, data):

    async def square(linear_speed=0.5, angular_speed=rad(45)):
        while world.robot.pose.x < 2:
            await world.robot.drive(linear_speed, 0)
            await asyncio.sleep(0)
        while world.robot.pose.yaw < rad(90):
            await world.robot.drive(0, angular_speed)
            await asyncio.sleep(0)
        while world.robot.pose.y < 2:
            await world.robot.drive(linear_speed, 0)
            await asyncio.sleep(0)
        while world.robot.pose.yaw < rad(180):
            await world.robot.drive(0, angular_speed)
            await asyncio.sleep(0)
        while world.robot.pose.x > 0:
            await world.robot.drive(linear_speed, 0)
            await asyncio.sleep(0)
        while world.robot.pose.yaw < rad(270):
            await world.robot.drive(0, angular_speed)
            await asyncio.sleep(0)
        while world.robot.pose.y > 0:
            await world.robot.drive(linear_speed, 0)
            await asyncio.sleep(0)
        await world.robot.drive(0, 0)

    if data == "square":
        world.robot.automate(square())


async def do_updates():
    while True:
        await sio.emit('world', world.dict())
        await asyncio.sleep(0.1)

tasks = []


@app.on_event("startup")
async def startup():
    loop = asyncio.get_event_loop()
    loop.set_debug(True)

    for actor in actors:
        tasks.append(task_logger.create_task(actor.run()))
    tasks.append(task_logger.create_task(do_updates()))


@app.on_event("shutdown")
async def shutdown():
    ic('shutting down')
    for task in tasks:
        task.cancel()
    ic('all tasks canceled')


@app.get("/")
def main():
    return {"status": "hello, I'm the robot system!", 'world': world.dict()}


@app.get("/world", response_model=World)
def get_world():
    return world


if __name__ == "__main__":
    uvicorn.run(
        "main:app", host="0.0.0.0", port=7000,
        reload=True, lifespan='on', forwarded_allow_ips='*', proxy_headers=True
    )
