#!/usr/bin/env python3
from nicegui import ui
import pylab as pl
from rosys.actors.pathplanning import generate_experiment, plot_tools as pt

robot_renderer, pose, goal, obstacle_map, small_obstacle_map, backward_to_goal = generate_experiment(5.0)

with ui.plot():
    pt.show_obstacle_map(obstacle_map)
    pl.gca().invert_yaxis()
    pl.autoscale(False)
    pt.plot_robot(robot_renderer, pose, 'C0')
    pt.plot_robot(robot_renderer, goal, 'C1')

ui.run()
