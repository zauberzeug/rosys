from pydantic import BaseModel
from pydantic.fields import Field
import serial
from line_reader import LineReader
from datetime import datetime, timedelta
import numpy as np
from easy_vector import Vector as V


class Pose(BaseModel):
    location: V
    orientation: int = Field(..., description='angle in degrees')

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
    pose: Pose = Pose(location=V(0, 0), orientation=0)
    width: float = 0.5

    def get_speed(self):
        print('....vvvvv.....', (self.drive.right - self.drive.left), self.width,
              (self.drive.right - self.drive.left) / self.width, flush=True)
        return Speed(
            linear=(self.drive.left + self.drive.right) / 2.0,
            angular=(self.drive.right - self.drive.left) / self.width
        )

    def do_drive(self):
        pass


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

    def do_drive(self,):
        line = "drive pw %.3f,%.3f" % (self.drive.left, self.drive.right)
        print(line, flush=True)
        checksum = 0
        for c in line:
            checksum ^= ord(c)
        line += '^%d\n' % checksum
        self.port.write(line.encode('utf-8'))
