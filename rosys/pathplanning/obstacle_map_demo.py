#!/usr/bin/env python3
import time

import numpy as np
import pylab as pl
from nicegui import ui

from rosys.geometry import Pose, Spline
from rosys.pathplanning import plot_tools as pt
from rosys.pathplanning.grid import Grid
from rosys.pathplanning.obstacle_map import ObstacleMap
from rosys.pathplanning.robot_renderer import RobotRenderer

grid = Grid((30, 40, 36), (0.45, -0.05, 4.0, 3.0))
obstacles = [
    [0.5, 1.5, 1.4, 0.1],
    [2.7, 1.5, 1.8, 0.1],
]
robot_renderer = RobotRenderer.from_size(0.77, 1.21, 0.445)

t = time.time()
obstacle_map = ObstacleMap.from_list(grid, obstacles, robot_renderer)
ui.label(f'{(time.time() - t) * 1000:.3f} ms')

start = [1.0, 0.5, 0]
end = [2.3, 0.9, np.deg2rad(90)]
spline = Spline.from_poses(
    Pose(x=start[0], y=start[1], yaw=start[2]),
    Pose(x=end[0], y=end[1], yaw=end[2]),
)

with ui.pyplot():
    pt.show_obstacle_map(obstacle_map)
    pl.autoscale(False)
    pt.plot_robot(robot_renderer, start, 'C3' if obstacle_map.test(*start) else 'C2')
    pt.plot_robot(robot_renderer, end, 'C3' if obstacle_map.test(*end) else 'C2')
    pt.plot_spline(spline, 'C3' if obstacle_map.test_spline(spline) else 'C2')

with ui.pyplot():
    pl.imshow(obstacle_map.dist_stack[:, :, 9], cmap=pl.cm.gray)  # pylint: disable=no-member

ui.run()
