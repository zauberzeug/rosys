from typing import Optional
import numpy as np
import scipy.interpolate
import time
from ...world import Point
from .obstacle_map import ObstacleMap


class DistanceMap:

    def __init__(self, obstacle_map: ObstacleMap, target: Point, deadline: Optional[float] = None):
        self.grid = obstacle_map.grid
        scaled_obstacle_map = obstacle_map.map

        self.INF = 1000000
        self.F = 100
        dx = self.F * self.grid.bbox[2] / self.grid.size[1]
        dy = self.F * self.grid.bbox[3] / self.grid.size[0]
        self.DX = int(dx)
        self.DY = int(dy)
        self.DD = int(np.sqrt(dx**2 + dy**2))

        scaled_inf = self.F * self.INF

        d = np.zeros(self.grid.size[:2], dtype=int)
        d.fill(scaled_inf)
        row, col = self.grid.to_grid(target.x, target.y)
        for r in [np.floor(row), np.ceil(row)]:
            for c in [np.floor(col), np.ceil(col)]:
                d[int(r), int(c)] = np.sqrt((row - r)**2 + (col - c)**2)

        old_sum = d.sum()
        while True:
            d_dx = d + self.DX
            d_dy = d + self.DY
            d_dd = d + self.DD
            d[1:-1, 1:-1] = np.minimum(
                d[1:-1, 1:-1],
                np.minimum(
                    np.minimum(
                        np.minimum(d_dy[:-2, 1:-1], d_dy[+2:, 1:-1]),
                        np.minimum(d_dx[1:-1, :-2], d_dx[1:-1, +2:])
                    ),
                    np.minimum(
                        np.minimum(d_dd[+2:, +2:], d_dd[+2:, :-2]),
                        np.minimum(d_dd[:-2, +2:], d_dd[:-2, :-2])
                    )
                )
            )
            d[scaled_obstacle_map] = scaled_inf
            new_sum = d.sum()
            if new_sum == old_sum:
                break
            old_sum = new_sum
            if deadline and time.time() > deadline:
                raise TimeoutError('distance map creation took too long')

        self.map = d.astype(float) / self.F
        gy, gx = np.gradient(self.map)

        rows = np.arange(self.grid.size[0])
        cols = np.arange(self.grid.size[1])
        xx, yy = self.grid.from_grid(rows, cols)
        self._interp = scipy.interpolate.interp2d(xx, yy, self.map, kind='linear')
        self._grad_x = scipy.interpolate.interp2d(xx, yy, gx, kind='linear')
        self._grad_y = scipy.interpolate.interp2d(xx, yy, gy, kind='linear')

        self.map[self.map >= self.INF] = np.inf

    def interpolate(self, x, y):
        result = self._interp(x, y)
        result[result >= self.INF / self.F] = np.inf
        return result

    def gradient(self, x, y):
        result_x = self._grad_x(x, y)
        result_y = self._grad_y(x, y)
        result_x[result_x <= -self.INF / self.F] = -np.inf
        result_y[result_y <= -self.INF / self.F] = -np.inf
        result_x[result_x >= self.INF / self.F] = np.inf
        result_y[result_y >= self.INF / self.F] = np.inf
        return result_x, result_y
