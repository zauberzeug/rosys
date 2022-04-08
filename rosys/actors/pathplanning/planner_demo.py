#!/usr/bin/env python3
import time
from uuid import uuid4

import numpy as np
import pylab as pl
from nicegui import ui
from rosys.actors.pathplanning import PlannerProcess
from rosys.actors.pathplanning import plot_tools as pt
from rosys.actors.pathplanning.robot_renderer import RobotRenderer
from rosys.world import Obstacle, Point, Pose, World

world = World()
obstacles = [Obstacle(
    id=str(uuid4()),
    outline=[Point(x=x, y=y), Point(x=x+w, y=y), Point(x=x+w, y=y+h), Point(x=x, y=y+h)]
) for x, y, w, h in [
    [1.5, 1.5, 0.2, 6.0],
    [1.5, 1.5, 5.0, 0.2],
    [6.3, 1.5, 0.2, 2.0],
    [4.5, 2.5, 0.2, 1.5],
    [5.5, 5.0, 0.2, 0.2],
]]
start = Pose()
goal = Pose(x=10, y=4, yaw=0)
planner = PlannerProcess(None, world.robot.shape.outline)
plot = ui.plot()


def run():
    t = time.time()
    planner.update_obstacle_map([], obstacles, [start, goal], deadline=time.time()+10.0)
    planner.update_distance_map(goal, deadline=time.time()+10.0)
    path = planner.search(goal=goal, start=start, backward=False, deadline=time.time()+10.0)

    with plot:
        pl.clf()
        pl.title(f'{time.time() - t:.3f} s')
        pt.show_distance_map(planner.state.distance_map)
        pt.show_obstacle_map(planner.state.obstacle_map)
        pl.gca().invert_yaxis()
        pl.autoscale(False)
        pt.plot_path(path, 'C0')
        [pl.plot(s.spline.end.x, s.spline.end.y, 'C0.') for s in path]
        robot_renderer = RobotRenderer(world.robot.shape.outline)
        pt.plot_robot(robot_renderer, (start.x, start.y, start.yaw), 'C2', lw=1)
        for step in path:
            yaw = step.spline.yaw(1) + np.pi if step.backward else step.spline.yaw(1)
            pt.plot_robot(robot_renderer, (step.spline.x(1), step.spline.y(1), yaw), 'C2', lw=1)


run()
ui.button('Run', on_click=run)

ui.run()
