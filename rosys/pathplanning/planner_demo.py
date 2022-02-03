#!/usr/bin/env python3
from nicegui import ui
import pylab as pl
import time
from uuid import uuid4
from rosys.world import Obstacle, Point, Pose, Robot, World
from rosys.pathplanning.planner import Planner
import rosys.pathplanning.plot_tools as pt

world = World(robot=Robot())
for x, y, w, h in [
    [1.5, 1.5, 0.2, 6.0],
    [1.5, 1.5, 5.0, 0.2],
    [6.3, 1.5, 0.2, 2.0],
    [4.5, 2.5, 0.2, 1.5],
    [5.5, 5.0, 0.2, 0.2],
]:
    id = str(uuid4())
    outline = [Point(x=x, y=y), Point(x=x+w, y=y), Point(x=x+w, y=y+h), Point(x=x, y=y+h)]
    world.obstacles[id] = Obstacle(id=id, outline=outline)

planner = Planner(world)

plot = ui.plot()


def run():
    t = time.time()
    planner.search(goal=Pose(x=10, y=4, yaw=0))
    print('path finding: %5.1f ms' % ((time.time() - t) * 1000), flush=True)

    with plot:
        pl.clf()
        pt.show_distance_map(planner.distance_map)
        pt.show_obstacle_map(planner.obstacle_map)
        pl.gca().invert_yaxis()
        pl.autoscale(False)
        pt.plot_path(planner.raw_path, 'C0')
        [pl.plot(s.target[0], s.target[1], 'C0.') for s in planner.raw_path]
        pt.plot_path(planner.path, 'C2', lw=2)
        [pl.plot(s.target[0], s.target[1], 'C2o') for s in planner.path]
        for step in planner.path:
            pt.plot_robot(world.robot.shape, (step.spline.x(1), step.spline.y(1), step.spline.yaw(1)), 'C2', lw=1)


run()
ui.button('Run', on_click=run)

ui.run()
