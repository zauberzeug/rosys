#!/usr/bin/env python3
from nicegui import ui

from rosys.geometry import Prism
from rosys.pathplanning import AreaManipulation, PathPlanner, area_object

# setup
path_planner = PathPlanner(Prism.default_robot_shape())
area_manipulation = AreaManipulation(path_planner)

# ui
with ui.card().classes('mx-auto'):
    area_manipulation.create_ui()
    with ui.scene(640, 480,
                  on_click=area_manipulation.handle_click,
                  on_drag_end=area_manipulation.handle_drag_end,
                  drag_constraints='z = 0'):
        area_object(path_planner, area_manipulation)

# start
ui.run(title='areas')
