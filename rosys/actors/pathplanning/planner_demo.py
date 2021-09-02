#!/usr/bin/env python3
from nicegui import ui
import pylab as pl
import plot_tools as pt
import time
from planner import Planner
from experiments import generate_experiment

experiment_id = [1.0, -1.0, 1.1, -1.1, 2.0, -2.0, 3.0, -3.1, -3.3, -4.0, 6.0][0]

t = time.time()
robot_renderer, pose, goal, obstacle_map, small_obstacle_map, backward_to_goal = generate_experiment(experiment_id)
ui.label("obstacle map: %5.1f ms" % ((time.time() - t) * 1000))

t = time.time()
planner = Planner(obstacle_map, small_obstacle_map, timeout=1.0)
planner.set_goal(goal, backward_to_goal=backward_to_goal)
planner.search(pose)
ui.label("path finding: %5.1f ms" % ((time.time() - t) * 1000))

with ui.plot():
    pl.title("Experiment %.1f (%s)" % (experiment_id, "backward" if backward_to_goal else "forward"))
    pt.show_distance_map(planner.distance_map)
    pt.show_obstacle_map(planner.obstacle_map)
    pl.gca().invert_yaxis()
    pl.autoscale(False)
    pt.plot_path(planner.raw_path, 'C0')
    [pl.plot(s.target[0], s.target[1], 'C0.') for s in planner.raw_path]
    pt.plot_path(planner.path, 'C2', lw=2)
    [pl.plot(s.target[0], s.target[1], 'C2o') for s in planner.path]
    pt.plot_robot(robot_renderer, pose, 'C2', lw=2)
    for step in planner.path:
        pt.plot_robot(robot_renderer, (step.spline.x(1), step.spline.y(1), step.spline.yaw(1)), 'C2', lw=1)

ui.run()
