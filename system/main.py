from fastapi import FastAPI
from fastapi_socketio import SocketManager
import asyncio
from robot import Robot, RealRobot, Speed, Drive

app = FastAPI()
sio = SocketManager(app=app)

try:
    robot = RealRobot()
except:
    robot = Robot()
    print('could not open serial -- using dummy robot', robot, flush=True)


@sio.on('drive_power')
def drive_power(sid, data):
    print(f'{sid} sends {data}', flush=True)
    robot.drive = Drive.parse_obj(data)


async def periodic():
    while True:
        speed = robot.get_speed()
        if speed:
            await sio.emit("odometry_speed", speed.dict())

        robot.send_drive()

        await asyncio.sleep(robot.idle_time)

task = None


@app.on_event("startup")
async def startup():
    global task
    print("---- startup", flush=True)
    loop = asyncio.get_event_loop()
    task = loop.create_task(periodic())


@app.on_event("shutdown")
async def shutdown():
    global task
    task.cancel()
    print("---- stop", flush=True)


@app.get("/api")
def main():
    return {"status": "hello, I'm the robot system!"}
