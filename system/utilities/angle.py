import numpy as np
from pydantic.main import BaseModel


class Angle(BaseModel):

    rad: float = 0

    @property
    def deg(self):
        return float(np.rad2deg(self.rad))


def deg(degrees):

    return Angle(rad=np.deg2rad(degrees))


def rad(radians):

    return Angle(rad=radians)
