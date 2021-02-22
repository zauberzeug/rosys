from utilities.angle import Angle, rad
from pydantic import BaseModel


class Velocity(BaseModel):

    linear: float = 0
    angular: Angle = rad(0)
