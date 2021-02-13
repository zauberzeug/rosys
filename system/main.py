from fastapi import FastAPI
from fastapi_socketio import SocketManager
import asyncio
from robot import Robot, RealRobot, Drive, Pose
import vectormath as vmath
from fastapi.encoders import jsonable_encoder


app = FastAPI()
sio = SocketManager(app=app)

try:
    robot = RealRobot()
except:
    robot = Robot()
    print('could not open serial -- using dummy robot', robot, flush=True)


@sio.on('drive_power')
def drive_power(sid, data):
    print(f'{sid} sends {data} to {robot}', flush=True)
    robot.drive = Drive.parse_obj(data)


custom_encoder = {
    vmath.Vector2: lambda v: {'x': v.x, 'y': v.y},
}


async def periodic():
    while True:
        speed = robot.get_speed()
        robot.pose.position += robot.pose.orientation * speed.linear
        await sio.emit("robot_pose", jsonable_encoder(robot.pose, custom_encoder=custom_encoder))

        robot.do_drive()

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
