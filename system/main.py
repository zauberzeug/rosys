from fastapi import FastAPI
from fastapi_socketio import SocketManager
import asyncio
from fastapi.encoders import jsonable_encoder
import task_logger
from world.world import World
from world.robot import Robot
from world.machine import Machine
import uvicorn

app = FastAPI()
sio = SocketManager(app=app)

machine = Machine(port="/dev/esp")
robot = Robot(machine=machine, width=0.5)
world = World(robot=robot)


@sio.on('connect')
async def on_connect(sid, _):
    await sio.emit('world', world.dict(), to=sid)


@sio.on('drive_power')
def on_drive_power(_, data):
    world.robot.power(data['left'], data['right'])


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
