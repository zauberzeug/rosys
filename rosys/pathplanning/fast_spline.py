from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..geometry import Pose


@dataclass
class FastSpline:
    start_x: float
    start_y: float
    start_yaw: float
    end_x: float
    end_y: float
    end_yaw: float
    backward: bool

    def __post_init__(self) -> None:
        distance = -0.5 if self.backward else 0.5
        distance *= np.sqrt((self.end_x - self.start_x)**2 + (self.end_y - self.start_y)**2)
        self.a = self.start_x
        self.e = self.start_y
        self.b = self.start_x + distance * np.cos(self.start_yaw)
        self.f = self.start_y + distance * np.sin(self.start_yaw)
        self.c = self.end_x - distance * np.cos(self.end_yaw)
        self.g = self.end_y - distance * np.sin(self.end_yaw)
        self.d = self.end_x
        self.h = self.end_y
        self.m = self.d - 3 * self.c + 3 * self.b - self.a
        self.n = self.c - 2 * self.b + self.a
        self.o = self.b - self.a
        self.p = self.h - 3 * self.g + 3 * self.f - self.e
        self.q = self.g - 2 * self.f + self.e
        self.r = self.f - self.e

    @staticmethod
    def from_poses(start: Pose, end: Pose, *, backward: bool = False) -> FastSpline:
        return FastSpline(start.x, start.y, start.yaw, end.x, end.y, end.yaw, backward)

    def x(self, t: np.ndarray) -> np.ndarray:
        return t**3 * self.d + 3 * t**2 * (1 - t) * self.c + 3 * t * (1 - t)**2 * self.b + (1 - t)**3 * self.a

    def y(self, t: np.ndarray) -> np.ndarray:
        return t**3 * self.h + 3 * t**2 * (1 - t) * self.g + 3 * t * (1 - t)**2 * self.f + (1 - t)**3 * self.e

    def gx(self, t: np.ndarray) -> np.ndarray:
        return 3 * (self.m * t**2 + 2 * self.n * t + self.o)

    def gy(self, t: np.ndarray) -> np.ndarray:
        return 3 * (self.p * t**2 + 2 * self.q * t + self.r)

    def yaw(self, t: np.ndarray) -> np.ndarray:
        return np.arctan2(self.gy(t), self.gx(t))
