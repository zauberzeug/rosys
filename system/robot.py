from pydantic import BaseModel
import serial
from line_reader import LineReader
from datetime import datetime
import numpy as np
import vectormath as vmath


class Pose(BaseModel):
    position: vmath.Vector2
    orientation: vmath.Vector2

    class Config:
        arbitrary_types_allowed = True


class Speed(BaseModel):
    linear: float = 0
    angular: float = 0


class Drive(BaseModel):
    left: float = 0
    right: float = 0


class Robot(BaseModel):
    idle_time: float = 1
    drive: Drive = Drive(left=0, right=0)
    pose: Pose = Pose(position=vmath.Vector2(0, 0), orientation=vmath.Vector2(0, 1))

    _speed: Speed = Speed(linear=0, angular=0)
    _time: datetime = datetime.now()

    def get_speed(self):
        return self._speed

    def do_drive(self):
        delta = datetime.now() - self._time
        self._speed.linear = self.drive.left * delta.seconds
        self._speed.angular = self.drive.right * delta.seconds
        print(self._speed, flush=True)


class RealRobot(Robot):

    def __init__(self):
        self.idle_time = 0.001
        self.port = serial.Serial("/dev/esp", baudrate=115200, timeout=0.5)
        self.line_reader = LineReader(self.port)

    def get_speed(self):
        try:
            line = self.line_reader.readline().decode('utf-8').strip('\n')
            if '^' in line:
                line, check = line.split('^')
                checksum = 0
                for c in line:
                    checksum ^= ord(c)
                if checksum != int(check):
                    return None
        except:
            return None
        print("                          ", line, flush=True)
        return Speed(linear=float(line.split()[1]), angular=float(line.split()[2]))

    def do_drive(self):
        line = "drive pw %.3f,%.3f" % (self.drive.left, self.drive.right)
        print(line, flush=True)
        checksum = 0
        for c in line:
            checksum ^= ord(c)
        line += '^%d\n' % checksum
        self.port.write(line.encode('utf-8'))
