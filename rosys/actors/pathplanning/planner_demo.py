#!/usr/bin/env python3
from __future__ import annotations

import time

import numpy as np
import pylab as pl
from nicegui import ui
from rosys.actors.pathplanning import plot_tools as pt
from rosys.actors.pathplanning.delaunay_planner import DelaunayPlanner
from rosys.actors.pathplanning.robot_renderer import RobotRenderer
from rosys.world import Area, Obstacle, Point, Pose

seed = np.random.randint(0, 1000)
seed = 0
np.random.seed(seed)

robot_outline = [(-0.5, -0.5), (0.5, -0.5), (0.75, 0), (0.5, 0.5), (-0.5, 0.5)]
areas = [Area(id='main', outline=[Point(x=-5, y=-5), Point(x=25, y=-5), Point(x=25, y=25), Point(x=-5, y=25)])]
obstacles = [
    Obstacle(id='0', outline=[Point(x=5, y=-5), Point(x=8, y=-5), Point(x=8, y=15), Point(x=5, y=15)]),
    Obstacle(id='1', outline=[Point(x=12, y=5), Point(x=15, y=5), Point(x=15, y=25), Point(x=12, y=25)]),
]
start = Pose(x=np.random.uniform(-4, 4), y=np.random.uniform(-4, 4), yaw=np.random.uniform(-np.pi, np.pi))
goal = Pose(x=np.random.uniform(16, 24), y=np.random.uniform(16, 24), yaw=np.random.uniform(-np.pi, np.pi))
planner = DelaunayPlanner(robot_outline)


def run():
    t = time.time()
    planner.update_map(areas, obstacles, [start, goal], deadline=time.time()+10.0)
    dt0 = time.time() - t

    t = time.time()
    path = planner.search(start, goal)
    dt1 = time.time() - t

    with plot:
        pl.clf()
        pl.title(f'map: {dt0:.3f} s, path: {dt1:.3f} s ({seed=})')
        pt.show_obstacle_map(planner.obstacle_map)
        pl.gca().invert_yaxis()
        pl.autoscale(False)
        pl.triplot(planner.tri_points[:, 0], planner.tri_points[:, 1], planner.tri_mesh.simplices, lw=0.1)

        pt.plot_path(path, 'C1')
        robot_renderer = RobotRenderer(robot_outline)
        pt.plot_robot(robot_renderer, (start.x, start.y, start.yaw), 'C0', lw=2)
        pt.plot_robot(robot_renderer, (goal.x, goal.y, goal.yaw), 'C0', lw=2)
        for step in path:
            for t in [0, 1]:
                yaw = step.spline.yaw(t) + np.pi if step.backward else step.spline.yaw(t)
                pt.plot_robot(robot_renderer, (step.spline.x(t), step.spline.y(t), yaw), 'C2', lw=1)


with ui.row():
    plot = ui.plot(figsize=(8, 8))
    ui.button('Re-run', on_click=run).props('icon=replay outline')
run()

ui.run()
