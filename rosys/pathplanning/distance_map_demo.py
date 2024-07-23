#!/usr/bin/env python3
import time

import numpy as np
import pylab as pl
from nicegui import ui

from rosys.geometry import Point
from rosys.pathplanning import plot_tools as pt
from rosys.pathplanning.distance_map import DistanceMap
from rosys.pathplanning.grid import Grid
from rosys.pathplanning.obstacle_map import ObstacleMap
from rosys.pathplanning.robot_renderer import RobotRenderer

grid = Grid((60, 80, 36), (0, 0, 16.0, 12.0))
obstacles = [
    [0.0, 6.0, 5.6, 0.4],
    [10.8, 6.0, 7.2, 0.4],
]
robot_renderer = RobotRenderer.from_size(0.77, 1.21, 0.445)
obstacle_map = ObstacleMap.from_list(grid, obstacles, robot_renderer)

target = Point(x=4.0, y=2.0)

t = time.time()
distance_map = DistanceMap(obstacle_map, target)
ui.label(f'{(time.time() - t) * 1000:5.3f} ms')

with ui.pyplot():
    pt.show_distance_map(distance_map)
    pt.show_obstacle_map(obstacle_map)
    extent = pt.bbox_to_extent(grid.bbox)

with ui.pyplot():
    rows = np.arange(0, grid.size[0] - 1, 0.2)
    cols = np.arange(0, grid.size[1] - 1, 0.2)
    xx, yy = grid.from_grid(rows, cols)  # pylint: disable=unpacking-non-sequence
    interp = distance_map.interpolate(xx, yy).reshape(len(rows), len(cols))
    pl.imshow(interp, cmap=pl.cm.gray, interpolation='nearest',  # pylint: disable=no-member
              extent=extent, clim=[0, 30])
    pt.show_obstacle_map(obstacle_map)
    pl.autoscale(False)
    for y in np.linspace(8.0, 11.0, 4):
        for x in np.linspace(2.0, 14.0, 30):
            dx, dy = distance_map.gradient(x, y)
            pl.plot(x, y, 'C2.', ms=3)
            pl.plot([x, x + dx], [y, y + dy], 'C2', lw=1)

with ui.pyplot():
    Gx = distance_map.gradient(xx, yy)[0].reshape(len(yy), len(xx))
    pl.imshow(Gx, cmap=pl.cm.gray, interpolation='nearest', extent=extent, clim=[-1, 1])  # pylint: disable=no-member
    pt.show_obstacle_map(obstacle_map)

ui.run()
