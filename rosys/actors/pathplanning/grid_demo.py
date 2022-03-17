#!/usr/bin/env python3
import numpy as np
from rosys.actors.pathplanning import Grid

grid = Grid((6, 8), (2.0, 1.0, 4.0, 3.0))
x, y = 2.5, 1.0
print(x, y)
row, col = grid.to_grid(x, y)
print(col, row)
x, y = grid.from_grid(row, col)
print(x, y)

grid = Grid((6, 8, 36), (2.0, 1.0, 4.0, 3.0))
x, y, yaw = 2.5, 1.0, np.pi / 2
print(x, y, yaw)
row, col, layer = grid.to_grid(x, y, yaw)
print(col, row, layer)
x, y, yaw = grid.from_grid(row, col, layer)
print(x, y, yaw)
