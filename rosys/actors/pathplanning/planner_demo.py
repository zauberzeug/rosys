#!/usr/bin/env python3
from nicegui import ui
import pylab as pl
import time
from uuid import uuid4
from rosys.actors.pathplanning import PlannerProcess, plot_tools as pt
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
goal = Pose(x=10, y=4, yaw=0)
planner = PlannerProcess(None, world.robot.shape.outline)
plot = ui.plot()


def run():
    t = time.time()
    planner.update_obstacle_map([], obstacles, [Pose(), goal], deadline=time.time()+3.0)
    planner.update_distance_map(goal, deadline=time.time()+3.0)
    path = planner.search(goal=goal, start=Pose(), backward=False, deadline=time.time()+3.0)
    print('path finding: %5.1f ms' % ((time.time() - t) * 1000), flush=True)

    with plot:
        pl.clf()
        pt.show_distance_map(planner.state.distance_map)
        pt.show_obstacle_map(planner.state.obstacle_map)
        pl.gca().invert_yaxis()
        pl.autoscale(False)
        pt.plot_path(path, 'C0')
        [pl.plot(s.spline.end.x, s.spline.end.y, 'C0.') for s in path]
        for step in path:
            pt.plot_robot(world.robot.shape, (step.spline.x(1), step.spline.y(1), step.spline.yaw(1)), 'C2', lw=1)


run()
ui.button('Run', on_click=run)

ui.run()
