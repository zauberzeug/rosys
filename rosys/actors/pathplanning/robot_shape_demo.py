#!/usr/bin/env python3
from nicegui import ui
import numpy as np
import pylab as pl
from robot_shape import RobotShape

robot_shape = RobotShape.from_size(0.77, 1.21, 0.445)
pose = (0, 0, np.deg2rad(30))

with ui.scene() as scene:
    scene.extrusion(robot_shape.outline, 0.1).move(*pose[:2]).rotate(0, 0, pose[2])

with ui.plot():
    pl.imshow(robot_shape.render(0.1, pose[2]), cmap=pl.cm.gray, interpolation='nearest')
    pl.gca().invert_yaxis()
    pl.fill(robot_shape.rendered_outline[:, 0],
            robot_shape.rendered_outline[:, 1], color='none', ec='C0')

ui.run()
