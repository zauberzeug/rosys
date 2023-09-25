#!/usr/bin/env python3
import pylab as pl
from nicegui import ui

from rosys.pathplanning import plot_tools as pt
from rosys.pathplanning.experiments import generate_experiment

robot_renderer, pose, goal, obstacle_map, backward_to_goal = generate_experiment(5.0)

with ui.pyplot():
    pt.show_obstacle_map(obstacle_map)
    pl.gca().invert_yaxis()
    pl.autoscale(False)
    pt.plot_robot(robot_renderer, pose, 'C0')
    pt.plot_robot(robot_renderer, goal, 'C1')

ui.run()
