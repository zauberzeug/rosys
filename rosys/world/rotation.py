from __future__ import annotations
from pydantic import BaseModel
import numpy as np
import cv2


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

    @staticmethod
    def from_rvec(rvec) -> Rotation:
        return Rotation(R=cv2.Rodrigues(rvec)[0].tolist())

    def __mul__(self, other) -> Rotation:
        return Rotation(R=np.dot(self.R, other.R).tolist())

    @property
    def T(self) -> Rotation:
        return Rotation(R=np.array(self.R).T.tolist())

    @property
    def euler(self) -> tuple[float]:
        return (
            np.arctan2(self.R[2][1], self.R[2][2]),
            np.arctan2(-self.R[2][0], np.sqrt(self.R[2][1]**2 + self.R[2][2]**2)),
            np.arctan2(self.R[1][0], self.R[0][0]),
        )

    @property
    def total_angle(self) -> float:
        # https://en.wikipedia.org/wiki/Rotation_matrix#Determining_the_angle
        return np.arccos((np.trace(self.R) - 1) / 2)

    def __repr__(self) -> str:
        return self.__str__()

    def __str__(self) -> str:
        return '%6.1f %6.1f %6.1f' % tuple(np.round(np.rad2deg(self.euler), 2))
