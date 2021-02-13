from fastapi import FastAPI
from fastapi_socketio import SocketManager
import asyncio
from robot import Robot, RealRobot, Drive, Pose
from easy_vector import Vector as V
from fastapi.encoders import jsonable_encoder
from datetime import date, datetime, timedelta

app = FastAPI()
sio = SocketManager(app=app)

try:
    robot = RealRobot()
except:
    robot = Robot()
    print('could not open serial -- using dummy robot', robot, flush=True)

fast_forward: int = 0


@sio.on('drive_power')
def on_drive_power(sid, data):
    print(f'{sid} sends drive_power {data} to {robot}', flush=True)
    robot.drive = Drive.parse_obj(data)


@sio.on('fast_forward')
def on_fast_forward(sid, data):
    global fast_forward
    print(f'{sid} sends fast_forward {data}', flush=True)
    fast_forward = int(data)


async def periodic():
    global fast_forward
    next_print = datetime.now()
    step = 0
    while True:
        step += 1
        speed = robot.get_speed()
        robot.pose.position += robot.pose.orientation * speed.linear

        robot.do_drive()

        if next_print < datetime.now():
            print(f'          step: {step}', flush=True)
            next_print = datetime.now() + timedelta(seconds=1)

        if fast_forward > 0:
            fast_forward -= 1
            continue

        await sio.emit("robot_pose", jsonable_encoder(robot.pose))
        await asyncio.sleep(robot.idle_time)

task = None


@ app.on_event("startup")
async def startup():
    global task
    loop = asyncio.get_event_loop()
    task = loop.create_task(periodic())


@ app.on_event("shutdown")
async def shutdown():
    global task
    task.cancel()


@ app.get("/api")
def main():
    return {"status": "hello, I'm the robot system!"}
