from fastapi import FastAPI
from fastapi_socketio import SocketManager
import asyncio
from robot import Robot, RealRobot, Drive
from easy_vector import Vector as V
from fastapi.encoders import jsonable_encoder
from datetime import datetime, timedelta
import task_logger

app = FastAPI()
sio = SocketManager(app=app)

robot = None


def reset():
    global robot
    try:
        robot = RealRobot()
    except:
        robot = Robot()
        print('could not open serial -- using dummy robot', robot, flush=True)


fast_forward: int = 0


@sio.on('connect')
async def on_connect(sid, env):
    await sio.emit('robot_pose', jsonable_encoder(robot.pose), to=sid)
    return True


@sio.on('drive_power')
def on_drive_power(sid, data):
    print(f'{sid} received drive_power {data}', flush=True)
    robot.drive = Drive.parse_obj(data)


sleep_task = None


@sio.on('fast_forward')
def on_fast_forward(sid, data):
    global fast_forward
    global sleep_task
    print(f'{sid} received fast_forward {data}', flush=True)
    fast_forward = int(data)
    sleep_task.cancel()


@sio.on('reset')
def on_reset(sid):
    print(f'{sid} received reset', flush=True)
    reset()


async def periodic():
    global fast_forward
    global sleep_task
    next_print = datetime.now()
    step = 0
    time = datetime.now()
    passed_time = timedelta(seconds=0)
    while True:
        speed = robot.get_speed()

        if fast_forward > 0:
            step += fast_forward
            passed_time += timedelta(seconds=robot.idle_time) * fast_forward
            fast_forward = 0
        else:
            passed_time += datetime.now() - time

        robot.pose.location += V.polar(1, robot.pose.orientation) * speed.linear * passed_time.seconds
        # robot should turn 180° when angular speed is 1 m/s
        new_angle = robot.pose.orientation + speed.angular * 180 * passed_time.seconds
        robot.pose.orientation = new_angle % 360
        robot.do_drive()
        time = datetime.now()
        passed_time = timedelta(seconds=0)

        if next_print < datetime.now():  # only print every second to not slow down critical serial communication
            print(f'          step: {step}, location {robot.pose.location}, passed_time: {passed_time}', flush=True)
            next_print = datetime.now() + timedelta(seconds=1)

        await sio.emit("robot_pose", jsonable_encoder(robot.pose))
        sleep_task = asyncio.ensure_future(asyncio.sleep(robot.idle_time))
        try:
            await sleep_task
        except:
            pass
        step += 1

task = None


@ app.on_event("startup")
async def startup():
    global task
    reset()
    loop = asyncio.get_event_loop()
    loop.set_debug(True)

    task = task_logger.create_task(periodic())


@ app.on_event("shutdown")
async def shutdown():
    global task
    task.cancel()


@ app.get("/api")
def main():
    return {"status": "hello, I'm the robot system!"}
