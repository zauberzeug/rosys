#!/usr/bin/env python3
from nicegui import ui
import pylab as pl
import plot_tools as pt
import time
from planner import Planner
from experiments import generate_experiment

experiment_id = [1.0, -1.0, 1.1, -1.1, 2.0, -2.0, 3.0, -3.1, -3.3, -4.0, 6.0][0]

t = time.time()
robot_shape, pose, goal, obstacle_map, small_obstacle_map, backward_to_goal = generate_experiment(experiment_id)
ui.label("obstacle map: %5.1f ms" % ((time.time() - t) * 1000))

t = time.time()
global_planner = Planner(obstacle_map, small_obstacle_map, timeout=1.0)
global_planner.set_goal(goal, backward_to_goal=backward_to_goal)
global_planner.search(pose)
ui.label("path finding: %5.1f ms" % ((time.time() - t) * 1000))

with ui.plot():
    pl.title("Experiment %.1f (%s)" % (experiment_id, "backward" if backward_to_goal else "forward"))
    pt.show_distance_map(global_planner.distance_map)
    pt.show_obstacle_map(global_planner.obstacle_map)
    pl.gca().invert_yaxis()
    pl.autoscale(False)
    pt.plot_path(global_planner.raw_path, 'C0')
    [pl.plot(s.target[0], s.target[1], 'C0.') for s in global_planner.raw_path]
    pt.plot_path(global_planner.path, 'C2', lw=2)
    [pl.plot(s.target[0], s.target[1], 'C2o') for s in global_planner.path]
    pt.plot_robot(robot_shape, pose, 'C2', lw=2)
    for step in global_planner.path:
        pt.plot_robot(robot_shape, (step.spline.x(1), step.spline.y(1), step.spline.yaw(1)), 'C2', lw=1)

ui.run()
