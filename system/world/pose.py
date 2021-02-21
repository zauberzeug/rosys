from utilities.angle import Angle
from pydantic import BaseModel


class Pose(BaseModel):

    x: float = 0
    y: float = 0
    yaw: Angle = 0
