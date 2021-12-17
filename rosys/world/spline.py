from __future__ import annotations
from pydantic import BaseModel
from typing import Optional
import numpy as np
from .pose import Pose
from .point import Point


class Spline(BaseModel):
    start: Point
    control1: Point
    control2: Point
    end: Point

    a: float = 0
    b: float = 0
    c: float = 0
    d: float = 0
    e: float = 0
    f: float = 0
    g: float = 0
    h: float = 0

    m: float = 0
    n: float = 0
    o: float = 0
    p: float = 0
    q: float = 0
    r: float = 0

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.a = self.start.x
        self.e = self.start.y
        self.b = self.control1.x
        self.f = self.control1.y
        self.c = self.control2.x
        self.g = self.control2.y
        self.d = self.end.x
        self.h = self.end.y

        self.m = self.d - 3 * self.c + 3 * self.b - self.a
        self.n = self.c - 2 * self.b + self.a
        self.o = self.b - self.a
        self.p = self.h - 3 * self.g + 3 * self.f - self.e
        self.q = self.g - 2 * self.f + self.e
        self.r = self.f - self.e

    def __repr__(self) -> str:
        return f'{type(self).__qualname__}(' + ' ~ '.join([
            f'{self.start.x:.3f},{self.start.y:.3f}',
            f'{self.control1.x:.3f},{self.control1.y:.3f}',
            f'{self.control2.x:.3f},{self.control2.y:.3f}',
            f'{self.end.x:.3f},{self.end.y:.3f}',
        ]) + ')'

    @staticmethod
    def from_poses(start: Pose, end: Pose, *, control_dist: Optional[float] = None, backward: bool = False) -> Spline:
        distance = 0.5 * (start.distance(end) if control_dist is None else control_dist) * (-1 if backward else 1)
        return Spline(
            start=start.point,
            control1=start.point.polar(distance, start.yaw),
            control2=end.point.polar(-distance, end.yaw),
            end=end.point,
        )

    def x(self, t: float) -> float:
        return t**3 * self.d + 3 * t**2 * (1 - t) * self.c + 3 * t * (1 - t)**2 * self.b + (1 - t)**3 * self.a

    def y(self, t: float) -> float:
        return t**3 * self.h + 3 * t**2 * (1 - t) * self.g + 3 * t * (1 - t)**2 * self.f + (1 - t)**3 * self.e

    def gx(self, t: float) -> float:
        return 3 * (self.m * t**2 + 2 * self.n * t + self.o)

    def ggx(self, t: float) -> float:
        return 6 * (self.m * t + self.n)

    def gy(self, t: float) -> float:
        return 3 * (self.p * t**2 + 2 * self.q * t + self.r)

    def ggy(self, t: float) -> float:
        return 6 * (self.p * t + self.q)

    def yaw(self, t: float) -> float:
        return np.arctan2(self.gy(t), self.gx(t))

    def pose(self, t: float) -> Pose:
        return Pose(x=self.x(t), y=self.y(t), yaw=self.yaw(t))

    def curvature(self, t: float) -> float:
        x_ = self.gx(t)
        y_ = self.gy(t)
        x__ = self.ggx(t)
        y__ = self.ggy(t)
        return (x_ * y__ - y_ * x__) / (x_**2 + y_**2)**(3/2)

    def max_curvature(self, t_min: float = 0.0, t_max: float = 1.0) -> float:
        poly = [
            (1296 * self.m * self.p**2 + 1296 * self.m**3) * self.q -
            1296 * self.n * self.p**3 - 1296 * self.m**2 * self.n * self.p,
            (1620 * self.m * self.p**2 + 1620 * self.m**3) * self.r + 3240 * self.m * self.p * self.q**2 + (3240 * self.m**2 * self.n - 3240 *
                                                                                                            self.n * self.p**2) * self.q - 1620 * self.o * self.p**3 + ((-1620 * self.m**2 * self.o) - 3240 * self.m * self.n**2) * self.p,
            (5184 * self.m * self.p * self.q + 1296 * self.n * self.p**2 + 6480 * self.m**2 * self.n) * self.r + 1296 * self.m * self.q**3 - 1296 * self.n * self.p * self.q**2 +
            ((-6480 * self.o * self.p**2) - 1296 * self.m**2 * self.o + 1296 * self.m * self.n**2) *
            self.q + ((-5184 * self.m * self.n * self.o) - 1296 * self.n**3) * self.p,
            1296 * self.m * self.p * self.r**2 + (1944 * self.m * self.q**2 + 6480 * self.n * self.p * self.q - 1296 * self.o * self.p**2 + 1296 * self.m**2 * self.o + 8424 * self.m *
                                                  self.n**2) * self.r - 8424 * self.o * self.p * self.q**2 - 6480 * self.m * self.n * self.o * self.q + ((-1296 * self.m * self.o**2) - 1944 * self.n**2 * self.o) * self.p,
            2592 * self.n * self.p * self.r**2 + (3888 * self.n * self.q**2 - 2592 * self.o * self.p * self.q + 2592 * self.m * self.n * self.o +
                                                  3888 * self.n**3) * self.r - 3888 * self.o * self.q**3 + ((-2592 * self.m * self.o**2) - 3888 * self.n**2 * self.o) * self.q,
            -324 * self.m * self.r**3 + (1944 * self.n * self.q + 324 * self.o * self.p) * self.r**2 + ((-1944 * self.o * self.q**2) - 324 *
                                                                                                        self.m * self.o**2 + 1944 * self.n**2 * self.o) * self.r - 1944 * self.n * self.o**2 * self.q + 324 * self.o**3 * self.p,
        ]
        roots = np.roots(poly)
        t = np.array([t0 for t0 in roots if np.isreal(t0) and t_min < t0 < t_max] + [t_min, t_max])

        k = np.real(self.curvature(t))
        idx = np.argmax(np.abs(k))
        return k[idx]

    def closest_point(self, x: float, y: float, t_min: float = 0.0, t_max: float = 1.0) -> float:
        poly = [
            6 * self.h**2 + ((-36 * self.g) + 36 * self.f - 12 * self.e) * self.h + 54 * self.g**2 + (36 * self.e - 108 * self.f) * self.g + 54 * self.f**2 - 36 * self.e * self.f + 6 * self.e**2 +
            6 * self.d**2 + ((-36 * self.c) + 36 * self.b - 12 * self.a) * self.d + 54 * self.c**2 +
            (36 * self.a - 108 * self.b) * self.c + 54 * self.b**2 - 36 * self.a * self.b + 6 * self.a**2,
            (30 * self.g - 60 * self.f + 30 * self.e) * self.h - 90 * self.g**2 + (270 * self.f - 120 * self.e) * self.g - 180 * self.f**2 + 150 * self.e * self.f - 30 * self.e**2 +
            (30 * self.c - 60 * self.b + 30 * self.a) * self.d - 90 * self.c**2 + (270 * self.b -
                                                                                   120 * self.a) * self.c - 180 * self.b**2 + 150 * self.a * self.b - 30 * self.a**2,
            (24 * self.f - 24 * self.e) * self.h + 36 * self.g**2 + (144 * self.e - 216 * self.f) * self.g + 216 * self.f**2 - 240 * self.e * self.f + 60 * self.e**2 +
            (24 * self.b - 24 * self.a) * self.d + 36 * self.c**2 + (144 * self.a - 216 *
                                                                     self.b) * self.c + 216 * self.b**2 - 240 * self.a * self.b + 60 * self.a**2,
            ((-6 * self.h) + 18 * self.g - 18 * self.f + 6 * self.e) * y + ((-6 * self.d) + 18 * self.c - 18 * self.b + 6 * self.a) * x + 6 * self.e * self.h + (54 * self.f - 72 * self.e) *
            self.g - 108 * self.f**2 + 180 * self.e * self.f - 60 * self.e**2 + 6 * self.a * self.d +
            (54 * self.b - 72 * self.a) * self.c - 108 * self.b**2 + 180 * self.a * self.b - 60 * self.a**2,
            ((-12 * self.g) + 24 * self.f - 12 * self.e) * y + ((-12 * self.c) + 24 * self.b - 12 * self.a) * x + 12 * self.e * self.g + 18 *
            self.f**2 - 60 * self.e * self.f + 30 * self.e**2 + 12 * self.a *
            self.c + 18 * self.b**2 - 60 * self.a * self.b + 30 * self.a**2,
            (6 * self.e - 6 * self.f) * y + (6 * self.a - 6 * self.b) * x + 6 *
            self.e * self.f - 6 * self.e**2 + 6 * self.a * self.b - 6 * self.a**2,
        ]

        roots = np.roots(poly)
        t = np.array([t0 for t0 in roots if np.isreal(t0) and t_min < t0 < t_max] + [t_min, t_max])

        sqr_d = (self.x(t) - x)**2 + (self.y(t) - y)**2

        return np.real(t[np.argmin(sqr_d)])

    def turning_points(self, t_min: float = 0.0, t_max: float = 1.0) -> list[float]:
        inner = self.m**2 * self.r**2 + ((4 * self.n**2 - 2 * self.m * self.o) * self.p - 4 * self.m * self.n * self.q) * \
            self.r + 4 * self.m * self.o * self.q**2 - 4 * self.n * self.o * self.p * self.q + self.o**2 * self.p**2
        if inner < 0:
            return np.array([])

        denominator = 2 * self.m * self.q - 2 * self.n * self.p
        if abs(denominator) < 1e-16:
            return np.array([])

        t = [
            +(np.sqrt(inner) - self.m * self.r + self.o * self.p) / denominator,
            -(np.sqrt(inner) + self.m * self.r - self.o * self.p) / denominator,
        ]
        return np.array([t_ for t_ in t if t_min <= t_ <= t_max])
