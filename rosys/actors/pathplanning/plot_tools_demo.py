#!/usr/bin/env python
from nicegui import ui
import pylab as pl
from rosys.world import Pose, Spline
from rosys.actors.pathplanning import Grid, ObstacleMap, RobotRenderer, plot_tools as pt

with ui.plot():
    pl.axis('equal')

    grid = Grid((3, 4, 36), (0.5, 1.5, 4.0, 3.0))
    o = [2.55, 2.60, 0.9, 0.89]
    robot_renderer = RobotRenderer.from_size(0.77, 1.21, 0.445)
    obstacle_map = ObstacleMap.from_list(grid, [o], robot_renderer)
    pt.plot_bbox(o, 'C3')
    pt.show_obstacle_map(obstacle_map)
    pl.gca().invert_yaxis()

    spline = Spline.from_poses(Pose(x=3, y=4, yaw=0.5), Pose(x=1, y=2, yaw=0), control_dist=0.5, backward=True)
    pl.plot(spline.start.x, spline.start.y, 'C1o')
    pl.plot(spline.end.x, spline.end.y, 'C1o')
    pl.plot(spline.control1.x, spline.control1.y, 'C2s')
    pl.plot(spline.control2.x, spline.control2.y, 'C2s')
    pt.plot_spline(spline, backward=True)
    pt.plot_robot(robot_renderer, (spline.x(0), spline.y(0), spline.yaw(0)), lw=2)

    x, y = 2, 2
    pl.plot(x, y, 'C4o')
    t = spline.closest_point(x, y)
    pl.plot(spline.x(t), spline.y(t), 'C4x')

ui.run()
