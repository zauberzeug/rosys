from __future__ import annotations
from pydantic import BaseModel
import numpy as np


class Rotation(BaseModel):

    R: list[list[float]]

    @staticmethod
    def zero():

        return Rotation(R=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    @staticmethod
    def from_euler(omega: float, phi: float, kappa: float) -> Rotation:

        Rx = np.array([[1, 0, 0], [0, np.cos(omega), -np.sin(omega)], [0, np.sin(omega), np.cos(omega)]])
        Ry = np.array([[np.cos(phi), 0, np.sin(phi)], [0, 1, 0], [-np.sin(phi), 0, np.cos(phi)]])
        Rz = np.array([[np.cos(kappa), -np.sin(kappa), 0], [np.sin(kappa), np.cos(kappa), 0], [0, 0, 1]])
        return Rotation(R=(Rz @ Ry @ Rx).tolist())

    def __mul__(self, other) -> Rotation:

        return Rotation(R=np.dot(self.R, other.R).tolist())

    @property
    def T(self) -> Rotation:

        return Rotation(R=np.array(self.R).T.tolist())
