#!/usr/bin/env python3
import numpy as np
import pylab as pl
from nicegui import ui

from rosys.pathplanning.robot_renderer import RobotRenderer

robot_renderer = RobotRenderer.from_size(0.77, 1.21, 0.445)
pose = (0, 0, np.deg2rad(30))

with ui.scene() as scene:
    scene.extrusion(robot_renderer.outline, 0.1).move(*pose[:2]).rotate(0, 0, pose[2])

with ui.pyplot():
    pl.imshow(robot_renderer.render(0.1, pose[2]), cmap=pl.cm.gray,  # pylint: disable=no-member
              interpolation='nearest')
    pl.gca().invert_yaxis()
    pl.fill(robot_renderer.rendered_outline[:, 0],
            robot_renderer.rendered_outline[:, 1], color='none', ec='C0')

ui.run()
