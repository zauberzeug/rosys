from utilities.angle import Angle
from pydantic import BaseModel


class Velocity(BaseModel):

    linear: float = 0
    angular: Angle = 0
