from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from pyquaternion import Quaternion


@dataclass(slots=True, kw_only=True)
class Rotation:
    R: list[list[float]]

    @staticmethod
    def zero() -> Rotation:
        return Rotation(R=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

    @staticmethod
    def from_euler(roll: float, pitch: float, yaw: float) -> Rotation:
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        return Rotation(R=(Rz @ Ry @ Rx).tolist())

    @staticmethod
    def from_quaternion(w: float, x: float, y: float, z: float) -> Rotation:
        return Rotation(R=Quaternion(w, x, y, z).rotation_matrix.tolist())

    @staticmethod
    def from_rvec(rvec) -> Rotation:
        return Rotation(R=cv2.Rodrigues(rvec)[0].tolist())

    def __mul__(self, other: Rotation) -> Rotation:
        return Rotation(R=np.dot(self.R, other.R).tolist())

    @property
    def matrix(self) -> np.ndarray:
        return np.array(self.R)

    @property
    def T(self) -> Rotation:
        return Rotation(R=np.transpose(self.R).tolist())

    @property
    def roll(self) -> float:
        return np.arctan2(self.R[2][1], self.R[2][2])

    @property
    def roll_deg(self) -> float:
        return np.rad2deg(self.roll)

    @property
    def pitch(self) -> float:
        return np.arctan2(-self.R[2][0], np.sqrt(self.R[2][1]**2 + self.R[2][2]**2))

    @property
    def pitch_deg(self) -> float:
        return np.rad2deg(self.pitch)

    @property
    def yaw(self) -> float:
        return np.arctan2(self.R[1][0], self.R[0][0])

    @property
    def yaw_deg(self) -> float:
        return np.rad2deg(self.yaw)

    @property
    def euler(self) -> tuple[float, float, float]:
        return self.roll, self.pitch, self.yaw

    @property
    def quaternion(self) -> tuple[float, float, float, float]:
        return Quaternion(matrix=np.array(self.R)).elements

    @property
    def total_angle(self) -> float:
        # https://en.wikipedia.org/wiki/Rotation_matrix#Determining_the_angle
        trace = np.trace(self.R)
        return 0 if np.isclose(trace, 3) else np.arccos((trace - 1) / 2)

    def __repr__(self) -> str:
        return self.__str__()

    def __str__(self) -> str:
        roll, pitch, yaw = np.round(np.rad2deg(self.euler), 2)
        return f'{roll:6.1f} {pitch:6.1f} {yaw:6.1f}'
