from fastapi import FastAPI
from fastapi_socketio import SocketManager
import asyncio
from pydantic import BaseModel

app = FastAPI()
sio = SocketManager(app=app)


class Drive(BaseModel):
    left: float = 0
    right: float = 0


class Robot(BaseModel):
    drive: Drive = Drive(x=0, y=0)


robot = Robot()


@sio.on('drive_power')
def drive_power(sid, data):
    print(f'{sid} sends {data}', flush=True)
    robot.drive = Drive.parse_obj(data)


async def periodic():
    while True:
        # TODO send serial command to robot
        print(f'controller power {robot.drive.left}, {robot.drive.right}', flush=True)
        await asyncio.sleep(1)  # TODO sleep only 10 ms or so

task = None


@app.on_event("startup")
async def startup():
    print("---- startup", flush=True)
    loop = asyncio.get_event_loop()
    task = loop.create_task(periodic())


@app.on_event("shutdown")
async def shutdown():
    task.cancel()
    print("---- stop", flush=True)


@app.get("/api")
def main():
    return {"status": "hello, I'm the robot system!"}
