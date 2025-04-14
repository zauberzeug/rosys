#!/usr/bin/env python3
import logging
import time

import numpy as np
import pylab as pl
from nicegui import app, ui

# pylint: disable=unused-import
from rosys.geometry import Point, Pose  # noqa: F401
from rosys.pathplanning import plot_tools as pt
from rosys.pathplanning.area import Area  # noqa: F401
from rosys.pathplanning.delaunay_planner import DelaunayPlanner
from rosys.pathplanning.obstacle import Obstacle  # noqa: F401
from rosys.pathplanning.planner_process import PlannerSearchCommand
from rosys.pathplanning.robot_renderer import RobotRenderer

robot_shape = ui.input('Robot shape', placeholder='[(x0, y0), (x1, y1), ...]') \
    .bind_value(app.storage.general, 'robot_shape') \
    .classes('w-full')
search_command = ui.textarea('Search command:',
                             placeholder='PlannerSearchCommand(...)') \
    .bind_value(app.storage.general, 'planner_search_command') \
    .classes('w-full')


def run() -> None:
    if not robot_shape.value or not search_command.value:
        return

    shape: list[tuple[float, float]] = eval(robot_shape.value)  # pylint: disable=eval-used
    cmd: PlannerSearchCommand = eval(search_command.value)  # pylint: disable=eval-used

    planner = DelaunayPlanner(shape)

    t = time.time()
    planner.update_map(cmd.areas, cmd.obstacles,
                       [cmd.start.point, cmd.goal.point],
                       deadline=time.time()+10.0)
    dt0 = time.time() - t

    t = time.time()
    try:
        path = planner.search(cmd.start, cmd.goal)
    except RuntimeError:
        logging.exception('could not find path')
        path = []
    dt1 = time.time() - t

    with plot:
        pl.clf()
        pl.title(f'map: {dt0:.3f} s, path: {dt1:.3f} s')
        pt.show_obstacle_map(planner.obstacle_map)
        pl.gca().invert_yaxis()
        pl.autoscale(False)
        assert planner.tri_points is not None
        assert planner.tri_mesh is not None
        pl.triplot(planner.tri_points[:, 0], planner.tri_points[:, 1],
                   planner.tri_mesh.simplices, lw=0.1)

        pt.plot_path(path, 'C1')
        robot_renderer = RobotRenderer(shape)
        pt.plot_robot(robot_renderer, (cmd.start.x, cmd.start.y, cmd.start.yaw),
                      'C0', lw=2)
        pt.plot_robot(robot_renderer, (cmd.goal.x, cmd.goal.y, cmd.goal.yaw),
                      'C0', lw=2)
        for step in path:
            for t in [0, 1]:
                yaw = step.spline.yaw(t) + np.pi if step.backward else step.spline.yaw(t)
                pt.plot_robot(robot_renderer,
                              (step.spline.x(t), step.spline.y(t), yaw),
                              'C2', lw=1)


with ui.row():
    plot = ui.pyplot(figsize=(8, 8))
    ui.button('Re-run', on_click=run).props('icon=replay outline')
run()

ui.run(title='Path Planning Analysis')
