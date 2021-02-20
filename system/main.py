from fastapi import FastAPI
from fastapi_socketio import SocketManager
import asyncio
from fastapi.encoders import jsonable_encoder
from datetime import datetime, timedelta
import task_logger
from world.world import World
from world.robot import Robot
from world.clock import Clock

app = FastAPI()
sio = SocketManager(app=app)

clock = Clock(interval=0.1)
robot = Robot(width=0.5)
world = World(clock=clock, robot=robot)


@sio.on('connect')
async def on_connect(sid, env):
    await sio.emit('robot_pose', jsonable_encoder(world.robot.pose), to=sid)
    return True


@sio.on('drive_power')
def on_drive_power(sid, data):
    print(f'{sid} received drive_power {data}', flush=True)
    world.robot.drive(data['m_per_s'], data['rad_per_s'])


running_world = None


@app.on_event("startup")
async def startup():
    global running_world
    loop = asyncio.get_event_loop()
    loop.set_debug(True)

    running_world = task_logger.create_task(world.run())


@app.on_event("shutdown")
async def shutdown():
    global running_world
    running_world.cancel()


@app.get("/api")
def main():
    return {"status": "hello, I'm the robot system!", 'world': world}


@app.get("/api/world", response_model=World)
def get_world():
    return world
