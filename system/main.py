from fastapi import FastAPI
from fastapi_socketio import SocketManager
import asyncio
from pydantic import BaseModel
import serial
from line_reader import LineReader

app = FastAPI()
sio = SocketManager(app=app)

port = serial.Serial("/dev/esp", baudrate=115200, timeout=0.5)
line_reader = LineReader(port)


class Speed(BaseModel):
    linear: float = 0
    angular: float = 0


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
        try:
            line = line_reader.readline().decode('utf-8').strip('\n')
            if '^' in line:
                line, check = line.split('^')
                checksum = 0
                for c in line:
                    checksum ^= ord(c)
                if checksum != int(check):
                    continue
        except:
            continue
        print("                          ", line, flush=True)
        linear = float(line.split()[1])
        angular = float(line.split()[2])
        await sio.emit("odometry_speed", Speed(linear=linear, angular=angular).dict())

        line = "drive pw %.3f,%.3f" % (robot.drive.left, robot.drive.right)
        print(line, flush=True)
        checksum = 0
        for c in line:
            checksum ^= ord(c)
        line += '^%d\n' % checksum
        port.write(line.encode('utf-8'))

        await asyncio.sleep(0.001)

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
