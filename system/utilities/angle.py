import numpy as np


class Angle(float):

    @property
    def deg(self):
        return float(np.rad2deg(self))

    @property
    def rad(self):
        return float(self)


def deg(degrees):

    return Angle(np.deg2rad(degrees))


def rad(radians):

    return Angle(radians)
