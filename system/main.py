from fastapi import FastAPI
from fastapi_socketio import SocketManager
import asyncio
from fastapi.encoders import jsonable_encoder
import task_logger
from world.world import World
from world.robot import Robot
from world.machine import Machine
from world.clock import Clock
import uvicorn

app = FastAPI()
sio = SocketManager(app=app)

clock = Clock(interval=0.01)
machine = Machine(port="/dev/esp")
robot = Robot(machine=machine, width=0.5)
world = World(clock=clock, robot=robot)


@sio.on('connect')
async def on_connect(sid, _):
    data = world.dict(exclude={"robot": {"machine": {"aioserial_instance"}}})
    await sio.emit('world', data, to=sid)


@sio.on('drive_power')
def on_drive_power(_, data):
    world.robot.power(data['left'], data['right'])


async def do_updates():
    while True:
        await sio.emit('robot_pose', jsonable_encoder(world.robot.pose))
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
    data = world.dict(exclude={"robot": {"machine": {"aioserial_instance"}}})
    return {"status": "hello, I'm the robot system!", 'world': data}


@app.get("/world", response_model=World)
def get_world():
    return world  # FIXME


if __name__ == "__main__":
    uvicorn.run(
        "main:app", host="0.0.0.0", port=80,
        reload=True, lifespan='on', forwarded_allow_ips='*', proxy_headers=True
    )
