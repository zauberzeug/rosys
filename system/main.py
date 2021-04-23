from fastapi import FastAPI
from fastapi_socketio import SocketManager
import asyncio
import task_logger
import uvicorn
from numpy import deg2rad as rad
from world.world import World
from world.robot import Robot
from world.machine import MockedMachine, SerialMachine

app = FastAPI()
sio = SocketManager(app=app)

try:
    machine = SerialMachine(port="/dev/esp")
except:
    machine = MockedMachine(width=0.5, realtime=True)

robot = Robot(machine=machine)
world = World(robot=robot)


@sio.on('connect')
async def on_connect(sid, _):
    await sio.emit('world', world.dict(), to=sid)


@sio.on('drive_power')
async def on_drive_power(_, data):
    await world.robot.power(data['left'], data['right'])


@sio.on('task')
async def on_task(_, data):

    async def square(linear_speed=0.5, angular_speed=rad(45)):
        while world.robot.pose.x < 2:
            await world.robot.drive(linear_speed, 0)
        while world.robot.pose.yaw < rad(90):
            await world.robot.drive(0, angular_speed)
        while world.robot.pose.y < 2:
            await world.robot.drive(linear_speed, 0)
        while world.robot.pose.yaw < rad(180):
            await world.robot.drive(0, angular_speed)
        while world.robot.pose.x > 0:
            await world.robot.drive(linear_speed, 0)
        while world.robot.pose.yaw < rad(270):
            await world.robot.drive(0, angular_speed)
        while world.robot.pose.y > 0:
            await world.robot.drive(linear_speed, 0)
        await world.robot.drive(0, 0)

    if data == "square":
        world.robot.automate(square())


async def do_updates():
    while True:
        await sio.emit('robot_pose', world.robot.pose.dict())
        await asyncio.sleep(0.1)


running_world = None
client_updates = None


@app.on_event("startup")
async def startup():
    global running_world
    global client_updates
    loop = asyncio.get_event_loop()
    loop.set_debug(True)

    running_world = task_logger.create_task(world.run())
    client_updates = task_logger.create_task(do_updates())


@app.on_event("shutdown")
async def shutdown():
    running_world.cancel()
    client_updates.cancel()


@app.get("/")
def main():
    return {"status": "hello, I'm the robot system!", 'world': world.dict()}


@app.get("/world", response_model=World)
def get_world():
    return world  # FIXME


if __name__ == "__main__":
    uvicorn.run(
        "main:app", host="0.0.0.0", port=80,
        reload=True, lifespan='on', forwarded_allow_ips='*', proxy_headers=True
    )
