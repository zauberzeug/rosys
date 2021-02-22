from utilities.angle import Angle, rad
from pydantic import BaseModel


class Pose(BaseModel):

    x: float = 0
    y: float = 0
    yaw: Angle = rad(0)

    def __str__(self):
        return f'({round(self.x,2)} {round(self.y,2)}) {self.yaw.deg}°'
