import numpy as np


class Angle(float):

    @property
    def deg(self):
        return float(np.rad2deg(self))

    @property
    def rad(self):
        return float(self)

    def __iadd__(self, other):
        return rad(self.rad + other.rad)

    def __add__(self, other):
        return rad(self.rad + other.rad)


def deg(degrees):

    return Angle(np.deg2rad(degrees))


def rad(radians):

    return Angle(radians)
