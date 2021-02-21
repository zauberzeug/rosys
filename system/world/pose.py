from utilities.angle import Angle, rad
from pydantic import BaseModel


class Pose(BaseModel):

    x: float = 0
    y: float = 0
    yaw: Angle = rad(0)
